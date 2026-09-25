"""EVALUATION-ONLY language and dialogue metrics for the Korean dialogue pilot.

Nothing here may feed back into a robot request, memory or wake-up. These are
language/format measurements; they say nothing about task efficiency.
"""
from __future__ import annotations

import re

HANGUL = re.compile(r'[가-힣ᄀ-ᇿ㄰-㆏]')
LATIN = re.compile(r'[A-Za-z]')
JSON_LITERALS = ('null', 'true', 'false')
IMAGE_LABELS = ('CURRENT OWN RGB', 'TOP_SW', 'TOP_NW', 'TOP_SE', 'TOP_NE', 'TOP_WEST', 'TOP_EAST', 'RGB', 'TOP')
KINDS = ('red', 'cyan', 'green', 'yellow', 'blue', 'orange', 'purple', 'white')
KO_COLOUR = r'(빨강|빨간|레드|청록|시안|하늘|초록|녹색|그린|노랑|노란|옐로)'


def literal_pattern(labels=(), extra=()):
    """Regex of tokens that must stay literal and are excluded from the Hangul ratio."""
    tokens = sorted({*labels, *extra, *IMAGE_LABELS, *JSON_LITERALS, *KINDS, 'r1', 'r2', 'r3'},
                    key=len, reverse=True)
    words = '|'.join(re.escape(t) for t in tokens)
    # zone letters only as standalone letters (A구역, 구역 A, A에 ...): not inside English words
    return re.compile(rf'(?<![A-Za-z0-9_-])(?:{words}|[ABC])(?![A-Za-z0-9_])')


def strip_literals(text, labels=(), extra=()):
    return literal_pattern(labels, extra).sub(' ', text)


def hangul_ratio(text, labels=(), extra=()):
    """Hangul letters / (Hangul + Latin letters) after literal tokens are removed.

    Returns None when no letters remain (silence or literal-only); silence is
    never counted as Korean success by callers.
    """
    rest = strip_literals(text or '', labels, extra)
    h, l = len(HANGUL.findall(rest)), len(LATIN.findall(rest))
    return None if h + l == 0 else h / (h + l)


def english_words(text, labels=(), extra=()):
    """Non-literal Latin words left in the text (code-switching evidence)."""
    return re.findall(r'[A-Za-z][A-Za-z\'-]*', strip_literals(text or '', labels, extra))


def id_issues(text, labels):
    """Mistranslated or corrupted literal ids in free text."""
    issues = []
    for m in re.finditer(r'(?<![A-Za-z0-9_])([A-Za-z]+)-(\d+)(?![0-9])', text or ''):
        if m.group(0) not in labels:
            issues.append({'kind': 'unknown_label', 'text': m.group(0)})
    for m in re.finditer(KO_COLOUR + r'\s*-?\s*\d', text or ''):
        issues.append({'kind': 'translated_label', 'text': m.group(0)})
    for m in re.finditer(r'로봇\s*\d|(?<![A-Za-z0-9])R[123](?![0-9])|(?<![A-Za-z0-9])r\s+[123](?![0-9])', text or ''):
        issues.append({'kind': 'robot_id_variant', 'text': m.group(0)})
    for m in re.finditer(r'(에이|비|씨)\s*구역|구역\s*(에이|비|씨)(?![가-힣])', text or ''):
        issues.append({'kind': 'translated_zone', 'text': m.group(0)})
    return issues


# Rule-based dialogue acts, multi-label. Korean and English cues.
ACT_RULES = {
    'claim': r'맡|가져가|가져갈|옮기겠|옮길게|옮깁니다|담당|제가 .*(하겠|할게)|선언|claim|taking|take |i will|i\'ll',
    'yield': r'양보|넘기|넘겨|포기|대신 .*(고르|선택)|다른 (상자|작업)|yield|give .* to|leave .* to|instead',
    'request': r'부탁|해 ?주세요|해 ?줘|해 ?주실|요청|주시겠|please|could you|can you',
    'agree': r'좋아|좋습니다|동의|알겠|확인했|그렇게 하|수락|ok\b|okay|agree|sounds good|confirmed',
    'refuse': r'거절|안 됩니다|불가|못 합니다|reject|refuse|cannot|can\'t',
    'question': r'\?|까요|나요|습니까|할래|괜찮을',
    'report': r'완료|배달했|배달 완료|놓았|도착|끝났|멈췄|중단|delivered|finished|stopped|done',
    'inform_obstacle': r'막혀|막힌|장애물|길을 막|경로가 겹|blocked|obstacle|in the way',
    'correct': r'정정|수정합니다|아니라|잘못|correction|actually',
    'standby': r'대기|기다리|할 일이 없|standing by|stand by|nothing (useful )?remains',
}


def dialogue_acts(text):
    if not text or not text.strip():
        return ['silence']
    low = text.lower()
    acts = [act for act, rule in ACT_RULES.items() if re.search(rule, low)]
    return acts or ['other']


def struct_acts(message):
    return ['silence'] if message is None else [message.get('act', 'other')]


def references(text, peer_utterance, peer_id, labels):
    """Does text mention the peer robot or an item/zone the peer's utterance named?"""
    if not text:
        return False
    if re.search(rf'(?<![A-Za-z0-9]){peer_id}(?![0-9])', text):
        return True
    named = {m for m in re.findall(r'[a-z]+-\d+', peer_utterance or '') if m in labels}
    return any(label in text for label in named)
