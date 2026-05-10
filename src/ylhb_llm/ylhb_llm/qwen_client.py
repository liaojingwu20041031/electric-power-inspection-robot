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

    def parse_sales_dialogue(
        self,
        text: str,
        model: str,
        timeout_sec: float,
        products: List[Dict[str, Any]],
        dialogue: Dict[str, Any],
    ) -> Dict[str, Any]:
        system_prompt = (
            '你是智慧零售比赛机器人的销售对话决策器。'
            '你的任务是根据用户中文口语、历史对话和商品清单，判断是否执行购买、推荐商品、追问、取消或未知。'
            '你不是只给一个商品，而是像真实销售员一样：如果用户需求明确但商品未明确，给出一个主推商品，'
            '同时给出1到2个相关备选商品。主推商品必须最符合用户当前需求，备选商品必须和需求相关。'
            '如果用户明确说出商品名或别名，action=execute_pick。'
            '如果用户表达需求但没有明确商品，action=propose_product。'
            '如果信息不足，action=ask_clarification。'
            '如果用户确认上一轮推荐，action=execute_pick，并选择当前主推商品。'
            '如果用户说换一个，避开 rejected_product_ids 和 last_product_id，优先从 related_products 或同类商品中重新推荐。'
            '如果用户说不要碳酸的、便宜点、健康点等约束，根据上下文和约束重新排序推荐。'
            '如果用户取消，action=cancel。'
            '不要处理前进、后退、左转、右转、停止、结算等控制类任务，这些会由系统外部处理。'
            '只能从给定商品清单中选择商品，禁止创造商品。'
            '输出必须是 JSON，不要 Markdown，不要额外解释。'
            '允许的 action：execute_pick/propose_product/ask_clarification/cancel/unknown。'
            '字段：task, action, need, primary_product_id, primary_product_name, related_products, '
            'reply_cn, confidence, requires_confirmation, reason_cn。'
            'related_products 中每项字段：product_id, product_name, reason_cn。'
        )
        user_payload = {
            'current_user_text': text,
            'dialogue': dialogue,
            'products': products,
            'allowed_actions': [
                'execute_pick',
                'propose_product',
                'ask_clarification',
                'cancel',
                'unknown',
            ],
        }
        out = self.chat_completion(
            model=model,
            messages=[
                {'role': 'system', 'content': system_prompt},
                {'role': 'user', 'content': json.dumps(user_payload, ensure_ascii=False)},
            ],
            timeout_sec=timeout_sec,
            temperature=0.0,
            extra_body={'enable_thinking': False},
        )
        return parse_json_object(out)

    def transcribe_audio(self, audio_path: str, model: str, timeout_sec: float) -> str:
        audio_url = self._audio_data_url(audio_path)
        endpoint = self._dashscope_generation_url()
        payload = {
            'model': model,
            'input': {
                'messages': [
                    {
                        'role': 'user',
                        'content': [
                            {'audio': audio_url},
                        ],
                    },
                ],
            },
        }
        body = self._post_dashscope_json(
            endpoint=endpoint,
            payload=payload,
            timeout_sec=timeout_sec,
            error_prefix=self._asr_error_prefix(endpoint, model, audio_path, audio_url),
        )
        try:
            parsed = json.loads(body)
        except json.JSONDecodeError as exc:
            raise QwenClientError(f'Unexpected DashScope ASR response: {body[:500]}') from exc

        text = self._extract_text(parsed).strip()
        if not text:
            raise QwenClientError(f'DashScope ASR response has no text: {body[:500]}')
        return text

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
        voice: str = 'Serena',
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
        body = self._post_dashscope_json(
            endpoint=endpoint,
            payload=payload,
            timeout_sec=timeout_sec,
            error_prefix='DashScope TTS',
        )

        try:
            parsed = json.loads(body)
        except json.JSONDecodeError as exc:
            raise QwenClientError(f'Unexpected DashScope TTS response: {body[:500]}') from exc

        audio_url = self._extract_audio_url(parsed)
        if not audio_url:
            raise QwenClientError(f'DashScope TTS response has no audio url: {body[:500]}')
        return self._download_url(audio_url, timeout_sec)

    def _post_dashscope_json(
        self,
        endpoint: str,
        payload: Dict[str, Any],
        timeout_sec: float,
        error_prefix: str,
    ) -> str:
        if not self.api_key:
            raise QwenClientError(f'{self.api_key_env} is not set')

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
                return response.read().decode('utf-8')
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')
            raise QwenClientError(f'{error_prefix} HTTP {exc.code}: {detail}') from exc
        except Exception as exc:
            raise QwenClientError(f'{error_prefix}: {exc}') from exc

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

    def _asr_error_prefix(self, endpoint: str, model: str, audio_path: str, audio_url: str) -> str:
        try:
            audio_size = os.path.getsize(audio_path)
        except OSError:
            audio_size = -1
        return (
            'DashScope ASR '
            f'endpoint={endpoint} model={model} audio_bytes={audio_size} '
            f'audio_prefix={audio_url[:22]}'
        )

    def _extract_text(self, parsed: Dict[str, Any]) -> str:
        candidates: List[str] = []
        self._collect_text_values(parsed.get('text'), candidates)
        self._collect_text_values(parsed.get('transcription'), candidates)

        output = parsed.get('output')
        if isinstance(output, dict):
            self._collect_text_values(output.get('text'), candidates)
            self._collect_text_values(output.get('transcription'), candidates)
            choices = output.get('choices')
            if isinstance(choices, list):
                for choice in choices:
                    if not isinstance(choice, dict):
                        continue
                    self._collect_text_values(choice.get('text'), candidates)
                    message = choice.get('message')
                    if isinstance(message, dict):
                        self._collect_text_values(message.get('content'), candidates)
                        self._collect_text_values(message.get('text'), candidates)
        return ' '.join(item for item in candidates if item).strip()

    def _collect_text_values(self, value: Any, candidates: List[str]) -> None:
        if isinstance(value, str):
            stripped = value.strip()
            if stripped:
                candidates.append(stripped)
            return
        if isinstance(value, dict):
            self._collect_text_values(value.get('text'), candidates)
            self._collect_text_values(value.get('transcription'), candidates)
            self._collect_text_values(value.get('content'), candidates)
            return
        if isinstance(value, list):
            for item in value:
                self._collect_text_values(item, candidates)

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
