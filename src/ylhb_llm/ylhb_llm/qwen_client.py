import base64
import json
import mimetypes
import os
import re
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional


class QwenClientError(RuntimeError):
    pass


class QwenClient:
    def __init__(self, base_url: str, api_key_env: str = 'DASHSCOPE_API_KEY') -> None:
        self.base_url = base_url.rstrip('/')
        self.api_key_env = api_key_env

    @property
    def api_key(self) -> str:
        return os.getenv(self.api_key_env, '')

    def available(self) -> bool:
        return bool(self.api_key)

    def chat_completion(
        self,
        model: str,
        messages: List[Dict[str, Any]],
        timeout_sec: float,
        temperature: float = 0.1,
        extra_body: Optional[Dict[str, Any]] = None,
    ) -> str:
        if not self.api_key:
            raise QwenClientError(f'{self.api_key_env} is not set')

        payload: Dict[str, Any] = {
            'model': model,
            'messages': messages,
            'temperature': temperature,
        }
        if extra_body:
            payload.update(extra_body)

        data = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        request = urllib.request.Request(
            self.base_url + '/chat/completions',
            data=data,
            headers={
                'Authorization': f'Bearer {self.api_key}',
                'Content-Type': 'application/json',
            },
            method='POST',
        )

        try:
            with urllib.request.urlopen(request, timeout=timeout_sec) as response:
                body = response.read().decode('utf-8')
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')
            raise QwenClientError(f'DashScope HTTP {exc.code}: {detail}') from exc
        except Exception as exc:
            raise QwenClientError(str(exc)) from exc

        try:
            parsed = json.loads(body)
            return parsed['choices'][0]['message']['content']
        except Exception as exc:
            raise QwenClientError(f'Unexpected DashScope response: {body[:500]}') from exc

    def analyze_image(
        self,
        image_path: str,
        model: str,
        timeout_sec: float,
        product_names: List[str],
    ) -> Dict[str, Any]:
        image_url = self._image_data_url(image_path)
        prompt = (
            '你是智慧零售机器人。请理解任务书图片，客观描述画面中的人物、表情、动作、'
            '思考气泡或文字提示，再推理用户的潜在需求，但不要直接决定最终商品。'
            '播报风格参考：“这张图像展示了一个看起来口渴的卡通男孩。他的舌头伸出，'
            '表情像是在说我好渴，思考气泡中有一杯水，暗示他想要喝水。”'
            '要求描述完整但不要繁琐，不要使用“肚子咕咕叫”“明显急需”等夸张脑补；'
            '如果画面没有明确信息，就说“可能”。请只输出 JSON，不要 Markdown。字段：'
            'description_cn: 适合语音播报的2到3个中文短句，先描述可见画面，再说明需求，'
            '总长度控制在45到90个汉字；'
            'need: 一个英文意图标签，例如 thirsty/hungry/energy/tissue/hygiene/drink/snack/nutrition；'
            'preferred_categories: 按推荐优先级排列的商品类别或商品名数组；'
            'avoid_categories: 明显不适合的类别数组；'
            'confidence: 0到1。'
            f'可选商品范围包括：{", ".join(product_names)}。'
        )
        content = [
            {'type': 'image_url', 'image_url': {'url': image_url}},
            {'type': 'text', 'text': prompt},
        ]
        text = self.chat_completion(
            model=model,
            messages=[{'role': 'user', 'content': content}],
            timeout_sec=timeout_sec,
            temperature=0.0,
            extra_body={'enable_thinking': False},
        )
        return parse_json_object(text)

    def parse_command(
        self,
        text: str,
        model: str,
        timeout_sec: float,
        product_names: List[str],
    ) -> Dict[str, Any]:
        prompt = (
            '你是智慧零售机器人任务解析器。请把用户口语命令解析成 JSON，不要 Markdown。'
            '字段：intent，取 pick_item/checkout/unknown；item_name；need；confidence；reply_cn。'
            f'商品范围：{", ".join(product_names)}。用户命令：{text}'
        )
        out = self.chat_completion(
            model=model,
            messages=[{'role': 'user', 'content': prompt}],
            timeout_sec=timeout_sec,
            temperature=0.0,
            extra_body={'enable_thinking': False},
        )
        return parse_json_object(out)

    def transcribe_audio(self, audio_path: str, model: str, timeout_sec: float) -> str:
        audio_url = self._audio_data_url(audio_path)
        content = [
            {'type': 'input_audio', 'input_audio': {'data': audio_url, 'format': 'wav'}},
            {'type': 'text', 'text': '请把这段中文语音转成文字，只输出转写文本。'},
        ]
        return self.chat_completion(
            model=model,
            messages=[{'role': 'user', 'content': content}],
            timeout_sec=timeout_sec,
            temperature=0.0,
            extra_body={'asr_options': {'enable_itn': False}},
        ).strip()

    def synthesize_speech(self, text: str, model: str, timeout_sec: float) -> Optional[bytes]:
        return self.synthesize_speech_bytes(
            text=text,
            model=model,
            timeout_sec=timeout_sec,
        )

    def synthesize_speech_bytes(
        self,
        text: str,
        model: str,
        timeout_sec: float,
        voice: str = 'Cherry',
        language_type: str = 'Chinese',
    ) -> Optional[bytes]:
        if not self.api_key:
            raise QwenClientError(f'{self.api_key_env} is not set')

        endpoint = self._dashscope_generation_url()
        payload = {
            'model': model,
            'input': {
                'text': text,
                'voice': voice,
                'language_type': language_type,
            },
        }
        data = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        request = urllib.request.Request(
            endpoint,
            data=data,
            headers={
                'Authorization': f'Bearer {self.api_key}',
                'Content-Type': 'application/json',
            },
            method='POST',
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout_sec) as response:
                body = response.read().decode('utf-8')
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')
            raise QwenClientError(f'DashScope TTS HTTP {exc.code}: {detail}') from exc
        except Exception as exc:
            raise QwenClientError(str(exc)) from exc

        try:
            parsed = json.loads(body)
        except json.JSONDecodeError as exc:
            raise QwenClientError(f'Unexpected DashScope TTS response: {body[:500]}') from exc

        audio_url = self._extract_audio_url(parsed)
        if not audio_url:
            raise QwenClientError(f'DashScope TTS response has no audio url: {body[:500]}')
        return self._download_url(audio_url, timeout_sec)

    def _image_data_url(self, image_path: str) -> str:
        mime = mimetypes.guess_type(image_path)[0] or 'image/png'
        with open(image_path, 'rb') as f:
            encoded = base64.b64encode(f.read()).decode('ascii')
        return f'data:{mime};base64,{encoded}'

    def _audio_data_url(self, audio_path: str) -> str:
        with open(audio_path, 'rb') as f:
            encoded = base64.b64encode(f.read()).decode('ascii')
        return f'data:audio/wav;base64,{encoded}'

    def _dashscope_generation_url(self) -> str:
        if '/compatible-mode/' in self.base_url:
            root = self.base_url.split('/compatible-mode/', 1)[0]
        else:
            root = self.base_url
        return root.rstrip('/') + '/api/v1/services/aigc/multimodal-generation/generation'

    def _extract_audio_url(self, parsed: Dict[str, Any]) -> str:
        output = parsed.get('output')
        if isinstance(output, dict):
            audio = output.get('audio')
            if isinstance(audio, dict) and isinstance(audio.get('url'), str):
                return audio['url']
            if isinstance(audio, str):
                return audio
            choices = output.get('choices')
            if isinstance(choices, list):
                for choice in choices:
                    if not isinstance(choice, dict):
                        continue
                    message = choice.get('message')
                    if isinstance(message, dict):
                        audio = message.get('audio')
                        if isinstance(audio, dict) and isinstance(audio.get('url'), str):
                            return audio['url']
        audio = parsed.get('audio')
        if isinstance(audio, dict) and isinstance(audio.get('url'), str):
            return audio['url']
        if isinstance(audio, str):
            return audio
        return ''

    def _download_url(self, url: str, timeout_sec: float) -> bytes:
        request = urllib.request.Request(url, method='GET')
        try:
            with urllib.request.urlopen(request, timeout=timeout_sec) as response:
                return response.read()
        except Exception as exc:
            raise QwenClientError(f'Failed to download TTS audio: {exc}') from exc


def parse_json_object(text: str) -> Dict[str, Any]:
    stripped = text.strip()
    if stripped.startswith('```'):
        stripped = re.sub(r'^```(?:json)?', '', stripped).strip()
        stripped = re.sub(r'```$', '', stripped).strip()
    try:
        value = json.loads(stripped)
    except json.JSONDecodeError:
        match = re.search(r'\{.*\}', stripped, flags=re.S)
        if not match:
            raise QwenClientError(f'Model did not return JSON: {text[:300]}')
        value = json.loads(match.group(0))
    if not isinstance(value, dict):
        raise QwenClientError('Model JSON response is not an object')
    return value
