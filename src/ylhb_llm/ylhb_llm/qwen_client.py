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
        payload: Dict[str, Any] = {'model': model, 'messages': messages, 'temperature': temperature}
        if extra_body:
            payload.update(extra_body)
        data = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        request = urllib.request.Request(
            self.base_url + '/chat/completions',
            data=data,
            headers={'Authorization': f'Bearer {self.api_key}', 'Content-Type': 'application/json'},
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

    def chat_tools(
        self,
        model: str,
        system_prompt: str,
        messages: List[Dict[str, Any]],
        tools: List[Dict[str, Any]],
        timeout_sec: float,
        temperature: float = 0.0,
        extra_body: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        if not self.api_key:
            raise QwenClientError(f'{self.api_key_env} is not set')
        payload_messages = [{'role': 'system', 'content': system_prompt}]
        payload_messages.extend(messages)
        payload: Dict[str, Any] = {
            'model': model,
            'messages': payload_messages,
            'temperature': temperature,
            'tools': tools,
            'parallel_tool_calls': False,
        }
        if extra_body:
            payload.update(extra_body)
        data = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        request = urllib.request.Request(
            self.base_url + '/chat/completions',
            data=data,
            headers={'Authorization': f'Bearer {self.api_key}', 'Content-Type': 'application/json'},
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
            message = json.loads(body)['choices'][0]['message']
        except Exception as exc:
            raise QwenClientError(f'Unexpected DashScope response: {body[:500]}') from exc
        return {
            'message': message,
            'content': message.get('content') or '',
            'tool_calls': message.get('tool_calls') or [],
        }

    def parse_inspection_command(self, text: str, model: str, timeout_sec: float) -> Dict[str, Any]:
        prompt = (
            '你是电力巡检机器人任务解析器。请把用户中文口语命令解析成 JSON，不要 Markdown。'
            '本系统面向电力巡检任务，不处理与巡检无关的消费类指令。'
            '字段：intent, target_id, target_name, destination, reply_cn, confidence, requires_ack。'
            'intent 只能从 start_inspection, pause_inspection, resume_inspection, cancel_inspection, '
            'manual_takeover, inspect_checkpoint, emergency_stop, inspection_query, unknown 中选择。'
            f'用户命令：{text}'
        )
        out = self.chat_completion(
            model=model,
            messages=[{'role': 'user', 'content': prompt}],
            timeout_sec=timeout_sec,
            temperature=0.0,
            extra_body={'enable_thinking': False},
        )
        return parse_json_object(out)

    def analyze_inspection_image(self, image_path: str, model: str, timeout_sec: float, prompt_hint: str = '') -> Dict[str, Any]:
        image_url = self._image_data_url(image_path)
        prompt = (
            '你是电力巡检机器人视觉分析助手。请根据图片识别巡检相关目标或异常。'
            '重点关注：人员、安全帽、开关/刀闸状态、表计/指示灯、漏油、火源、烟雾、异物、障碍物。'
            '请只输出 JSON，不要 Markdown。字段：summary_cn, findings, alarms, confidence。'
            'findings 每项字段：target_type, target_name, state, bbox_hint, confidence, description_cn。'
            'alarms 每项字段：level, type, description_cn。'
            f'补充提示：{prompt_hint}'
        )
        content = [{'type': 'image_url', 'image_url': {'url': image_url}}, {'type': 'text', 'text': prompt}]
        out = self.chat_completion(
            model=model,
            messages=[{'role': 'user', 'content': content}],
            timeout_sec=timeout_sec,
            temperature=0.0,
            extra_body={'enable_thinking': False},
        )
        return parse_json_object(out)

    def transcribe_audio(self, audio_path: str, model: str, timeout_sec: float) -> str:
        audio_url = self._audio_data_url(audio_path)
        endpoint = self._dashscope_generation_url()
        payload = {'model': model, 'input': {'messages': [{'role': 'user', 'content': [{'audio': audio_url}]}]}}
        body = self._post_dashscope_json(endpoint, payload, timeout_sec, self._asr_error_prefix(endpoint, model, audio_path, audio_url))
        try:
            parsed = json.loads(body)
        except json.JSONDecodeError as exc:
            raise QwenClientError(f'Unexpected DashScope ASR response: {body[:500]}') from exc
        text = self._extract_text(parsed).strip()
        if not text:
            raise QwenClientError(f'DashScope ASR response has no text: {body[:500]}')
        return text

    def synthesize_speech(self, text: str, model: str, timeout_sec: float) -> Optional[bytes]:
        return self.synthesize_speech_bytes(text=text, model=model, timeout_sec=timeout_sec)

    def synthesize_speech_bytes(self, text: str, model: str, timeout_sec: float, voice: str = 'Serena', language_type: str = 'Chinese') -> Optional[bytes]:
        if not self.api_key:
            raise QwenClientError(f'{self.api_key_env} is not set')
        endpoint = self._dashscope_generation_url()
        payload = {'model': model, 'input': {'text': text, 'voice': voice, 'language_type': language_type}}
        body = self._post_dashscope_json(endpoint, payload, timeout_sec, 'DashScope TTS')
        try:
            parsed = json.loads(body)
        except json.JSONDecodeError as exc:
            raise QwenClientError(f'Unexpected DashScope TTS response: {body[:500]}') from exc
        audio_url = self._extract_audio_url(parsed)
        if not audio_url:
            raise QwenClientError(f'DashScope TTS response has no audio url: {body[:500]}')
        return self._download_url(audio_url, timeout_sec)

    def _post_dashscope_json(self, endpoint: str, payload: Dict[str, Any], timeout_sec: float, error_prefix: str) -> str:
        if not self.api_key:
            raise QwenClientError(f'{self.api_key_env} is not set')
        data = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        request = urllib.request.Request(endpoint, data=data, headers={'Authorization': f'Bearer {self.api_key}', 'Content-Type': 'application/json'}, method='POST')
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
        return f'DashScope ASR endpoint={endpoint} model={model} audio_bytes={audio_size} audio_prefix={audio_url[:22]}'

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
            for key in ('text', 'transcription', 'content'):
                self._collect_text_values(value.get(key), candidates)
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
                    if isinstance(choice, dict):
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
