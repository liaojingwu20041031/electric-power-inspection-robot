"""Minimal OpenAI-compatible client used only by the Agent planner."""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from typing import Any, Dict, List


class OpenAIToolClientError(RuntimeError):
    pass


class OpenAIToolClient:
    def __init__(
        self,
        provider_name: str,
        base_url: str,
        api_key_env: str,
        api_key_required: bool,
        chat_path: str = '/chat/completions',
        models_path: str = '/models',
        extra_body_json: str = '{}',
    ) -> None:
        self.provider_name = provider_name.strip() or 'planner'
        self.base_url = base_url.rstrip('/')
        self.api_key_env = api_key_env.strip()
        self.api_key_required = bool(api_key_required)
        self.chat_path = '/' + chat_path.lstrip('/')
        self.models_path = '/' + models_path.lstrip('/')
        try:
            self.extra_body = json.loads(extra_body_json or '{}')
        except json.JSONDecodeError as exc:
            raise ValueError('planner_extra_body_json must be an object') from exc
        if not isinstance(self.extra_body, dict):
            raise ValueError('planner_extra_body_json must be an object')

    @property
    def api_key(self) -> str:
        return os.environ.get(self.api_key_env, '') if self.api_key_env else ''

    def available(self) -> bool:
        return bool(self.base_url and (not self.api_key_required or self.api_key))

    def endpoint(self, path: str) -> str:
        return self.base_url + path

    def chat_tools(
        self,
        model: str,
        system_prompt: str,
        messages: List[Dict[str, Any]],
        tools: List[Dict[str, Any]],
        timeout_sec: float,
        temperature: float = 0.0,
        tool_choice: str = 'auto',
        extra_body: Dict[str, Any] | None = None,
    ) -> Dict[str, Any]:
        if not self.available():
            required = self.api_key_env or 'planner API key'
            raise OpenAIToolClientError(f'{required} is not set')
        payload: Dict[str, Any] = {
            'model': model,
            'messages': [{'role': 'system', 'content': system_prompt}, *messages],
            'temperature': temperature,
            'tools': tools,
            'parallel_tool_calls': False,
            'tool_choice': tool_choice,
            **self.extra_body,
        }
        if extra_body:
            payload.update(extra_body)
        message = self._post_json(self.chat_path, payload, timeout_sec)['choices'][0]['message']
        if not isinstance(message, dict):
            raise OpenAIToolClientError(f'Unexpected {self.provider_name} response: assistant message is invalid')
        return {
            'message': message,
            'content': message.get('content') or '',
            'tool_calls': message.get('tool_calls') or [],
        }

    def _post_json(self, path: str, payload: Dict[str, Any], timeout_sec: float) -> Dict[str, Any]:
        headers = {'Content-Type': 'application/json'}
        if self.api_key:
            headers['Authorization'] = f'Bearer {self.api_key}'
        request = urllib.request.Request(
            self.endpoint(path), data=json.dumps(payload, ensure_ascii=False).encode('utf-8'),
            headers=headers, method='POST',
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout_sec) as response:
                body = response.read().decode('utf-8')
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')
            raise OpenAIToolClientError(f'{self.provider_name} HTTP {exc.code}: {detail}') from exc
        except (OSError, urllib.error.URLError) as exc:
            raise OpenAIToolClientError(f'{self.provider_name}: {exc}') from exc
        try:
            parsed = json.loads(body)
            if not isinstance(parsed, dict):
                raise ValueError('response is not an object')
            return parsed
        except (ValueError, json.JSONDecodeError) as exc:
            raise OpenAIToolClientError(f'Unexpected {self.provider_name} response: {body[:500]}') from exc
