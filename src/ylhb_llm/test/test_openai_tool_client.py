import json

from ylhb_llm.openai_tool_client import OpenAIToolClient


def test_local_provider_without_key_omits_authorization(monkeypatch):
    seen = {}

    class Response:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self):
            return json.dumps({'choices': [{'message': {'role': 'assistant', 'content': 'ok'}}]}).encode()

    def fake_urlopen(request, timeout):
        seen['headers'] = dict(request.header_items())
        seen['body'] = json.loads(request.data.decode())
        seen['timeout'] = timeout
        return Response()

    monkeypatch.setattr('urllib.request.urlopen', fake_urlopen)
    client = OpenAIToolClient(
        provider_name='local', base_url='http://127.0.0.1:1234/v1',
        api_key_env='', api_key_required=False, extra_body_json='{}',
    )

    response = client.chat_tools(
        model='local-model', system_prompt='system', messages=[], tools=[], timeout_sec=2.0,
    )

    assert client.available()
    assert response['message']['content'] == 'ok'
    assert 'Authorization' not in seen['headers']
    assert seen['body']['parallel_tool_calls'] is False
    assert seen['body']['tool_choice'] == 'auto'


def test_configured_extra_body_is_sent_with_tool_call(monkeypatch):
    class Response:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self):
            return b'{"choices":[{"message":{"role":"assistant","content":"ok"}}]}'

    body = {}
    monkeypatch.setattr(
        'urllib.request.urlopen',
        lambda request, timeout: body.update(json.loads(request.data.decode())) or Response(),
    )
    client = OpenAIToolClient(
        provider_name='example', base_url='https://example.invalid/v1',
        api_key_env='', api_key_required=False, extra_body_json='{"feature": false}',
    )

    client.chat_tools(model='m', system_prompt='s', messages=[], tools=[], timeout_sec=1.0)

    assert body['feature'] is False
