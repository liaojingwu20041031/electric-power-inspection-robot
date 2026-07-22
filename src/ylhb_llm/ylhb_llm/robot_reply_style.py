import re


BAD_TONE_WORDS = ('亲', '哦', '我觉得吧')

REPLIES = {
    'emergency_stop': ('收到，已急停。', 10, True),
    'motion_stop': ('已停止短时运动。', 5, False),
    'patrol_start': ('收到，开始巡逻。', 5, False),
    'patrol_pause': ('收到，暂停巡逻。', 5, False),
    'patrol_resume': ('收到，继续巡逻。', 5, False),
    'patrol_cancel': ('收到，取消巡逻。', 5, False),
    'capability_query': ('我是电力巡检机器人，可以执行巡逻控制、短时运动和状态查询。', 5, False),
    'debug_query': ('我已收到问题。请检查麦克风、网络和日志状态。', 5, False),
    'unknown_local': ('我还不能确定这条指令，请换一种说法，或查看语音和系统状态。', 5, False),
}


def speak(intent: str, text: str = '') -> dict:
    reply, priority, interrupt = REPLIES.get(intent, (text, 5, False))
    return {
        'reply_key': f'command.{intent}',
        'text': sanitize_reply(text or reply),
        'priority': priority,
        'interrupt': interrupt,
    }


def sanitize_reply(text: str) -> str:
    cleaned = str(text)
    for word in BAD_TONE_WORDS:
        cleaned = cleaned.replace(word, '')
    return cleaned.strip()


def prepare_speech_text(text: str, max_chars: int = 60, max_sentences: int = 2) -> str:
    original = str(text or '').strip()
    cleaned = re.sub(r'```[\s\S]*?```', ' ', original)
    cleaned = re.sub(r'(?m)^\s{0,3}#{1,6}\s+.*$', ' ', cleaned)
    cleaned = re.sub(r'!\[[^]]*]\([^)]*\)', ' ', cleaned)
    cleaned = re.sub(r'\[([^]]+)]\([^)]*\)', r'\1', cleaned)
    cleaned = re.sub(r'https?://\S+|www\.\S+', ' ', cleaned)
    cleaned = re.sub(r'[（(][^）)]*(?:=|[A-Za-z_]{2})[^）)]*[）)]', '', cleaned)
    cleaned = re.sub(r'[：:]\s*\n+', '。\n', cleaned)
    cleaned = re.sub(r'(?m)^\s*\d+[.、)]\s*', '', cleaned)
    cleaned = re.sub(r'\n+\s*[-+*]\s*', '。', cleaned)
    cleaned = re.sub(r'\n+', '。', cleaned)
    cleaned = re.sub(
        r'<[^>]+>|\b(?:tool_name|tool_call_id|assistant_chat_[\w-]+|bringup|navigation|'
        r'perception|patrol_executor|ready|running|idle|unknown|failed|succeeded|'
        r'[a-z][a-z0-9]*(?:_[a-z0-9]+)+)\b',
        ' ', cleaned)
    cleaned = cleaned.replace('=', ' ')
    cleaned = re.sub(r'[`#>*_~|]+', ' ', cleaned)
    cleaned = re.sub(r'。{2,}', '。', cleaned)
    cleaned = re.sub(r'\s+([，。！？；：])', r'\1', cleaned)
    cleaned = re.sub(r'\s+', ' ', cleaned).strip()
    if not cleaned:
        return '暂时没有可播报的内容。'

    sentences = [item.strip() for item in re.findall(r'[^。！？]+[。！？]?', cleaned) if item.strip()]
    sentences = [item if item[-1] in '。！？' else item + '。' for item in sentences]
    useful = [
        item for item in sentences
        if not item.rstrip('。！？').endswith('如下')
        and '请按以下步骤操作' not in item
    ]
    if useful:
        sentences = useful
    complete = ''.join(sentences)
    sentence_limit = max(1, int(max_sentences))
    if len(sentences) <= sentence_limit and len(complete) <= max_chars:
        return complete
    selected = ''
    for sentence in sentences[:sentence_limit]:
        if len(selected + sentence) > max_chars:
            break
        selected += sentence
    if selected:
        return selected
    first = sentences[0].rstrip('。！？')
    excerpt = first[:max(1, max_chars - 1)]
    boundary = max(excerpt.rfind(mark) for mark in '，；：')
    if boundary >= 12:
        excerpt = excerpt[:boundary]
    return excerpt.rstrip('，、；： ') + '。'
