"""EVALUATION-ONLY language and dialogue metrics for the Korean dialogue pilot.

Nothing here may feed back into a robot request, memory or wake-up. These are
language/format measurements; they say nothing about task efficiency.

Dialogue-act rules are versioned: `v1` is frozen because it labelled
experiments/2026-09-25-zone-dialogue-ko-pilot/results.json (schema
...results.v1); `v2` is the 2026-09-26 review fix (word boundaries, negation,
act vs mention). Always report which version produced a label.
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


# ---------------------------------------------------------------- dialogue acts
#
# One vocabulary, two rule versions. `ACT_LABELS` is the full free-text
# vocabulary; `STRUCTURED_ACTS` is the V3 schema enum
# (harness.zone_dialogue_ko.ACTS, checked by a test). `propose` and `standby`
# exist only in free text, so V3 and free-text act distributions are NOT
# directly comparable; report the two vocabularies separately.
#
# v1  the rules that produced experiments/2026-09-25-zone-dialogue-ko-pilot/
#     results.json (schema ...results.v1). Kept byte-identical for that record:
#     substring cues, so a *mention* counts as the act ("Accepted proposal."
#     -> propose, "disagree" -> agree).
# v2  2026-09-26 review fix: word boundaries on Latin cues, explicit
#     suppression spans for negation and for mentions of someone else's act.
#     Each act is a (cue, suppress) pair; a cue match counts only when no
#     suppress match overlaps it.
ACT_RULES_VERSIONS = ('v1', 'v2')
ACT_RULES_VERSION = 'v2'
ACT_LABELS = ('claim', 'propose', 'yield', 'request', 'agree', 'refuse', 'question', 'report',
              'inform_obstacle', 'correct', 'standby')
STRUCTURED_ACTS = ('claim', 'request', 'inform_obstacle', 'report', 'agree', 'yield', 'question',
                   'correct', 'refuse')
ACTS_FREE_TEXT_ONLY = ('propose', 'standby')

ACT_DEFINITIONS = {
    'claim': '화자가 특정 상자·작업을 자기 것으로 가져간다는 의지 표명. 남을 배정하는 서술(r2 takes A)은 제외.',
    'propose': '배정안을 제시·제출·확정 제출하는 발화. 남의 제안을 언급·수락만 하는 발화는 제외.',
    'yield': '자기 몫·후보를 상대에게 넘기는 발화.',
    'request': '상대의 행동을 요구·권유하는 발화(청유형 포함).',
    'agree': '상대의 제안·선택을 받아들이는 발화. 부정형(동의하지 않음, disagree)은 제외.',
    'refuse': '거절·불가 표명. 부정형 동의도 여기에 들어간다.',
    'question': '질문 형식의 발화.',
    'report': '이미 일어난 완료·도착·중단 보고. 예정·조건부(완료하면)는 제외.',
    'inform_obstacle': '통행 방해·장애물 관찰 보고.',
    'correct': '앞선 발화·정보를 정정하는 발화.',
    'standby': '할 일이 없거나 기다린다는 표명.',
}

# Rule set v1 — FROZEN. Do not edit: results.v1 act counts were produced by it.
ACT_RULES_V1 = {
    'claim': r'맡겠|맡을|가져가겠|가져갈게|가져가도|가져오겠|옮기겠|옮길게|운반하겠|운반할게|배달하겠|전달하겠|담당하겠|'
             r'claim|taking|take |i will|i\'ll',
    'propose': r'제안합니다|제안할게|제안을 올|propos',
    'yield': r'양보|넘기|넘겨|포기|yield|give .* to|leave .* to',
    'request': r'부탁|해 ?주세요|해 ?줘|줘[.!]?$|올려줘|주실|요청|주시겠|please|could you|can you',
    'agree': r'좋아|좋습니다|동의합니다|동의해요|알겠|확인했|그렇게 하|수락합니다|수락할게|ok\b|okay|agree|sounds good|accept',
    'refuse': r'거절|안 됩니다|불가|못 합니다|reject|refuse|cannot|can\'t',
    'question': r'\?|까요|나요|습니까|할래|괜찮을',
    'report': r'완료|배달했|놓았|도착|끝났|멈췄|중단|delivered|finished|stopped|done',
    'inform_obstacle': r'막혀|막힌|장애물|길을 막|경로가 겹|blocked|obstacle|in the way',
    'correct': r'정정|수정합니다|아니라|잘못|correction|actually',
    'standby': r'대기|기다리|할 일이 없|standing by|stand by|nothing (useful )?remains|waiting',
}
ACT_RULES = ACT_RULES_V1          # backwards-compatible name for the frozen v1 rules

# Negated agreement: suppresses `agree` and is itself a `refuse` cue (v2 only).
NEG_AGREE = (r'동의하지\s*않|동의할\s*수\s*없|수락하지\s*않|수락할\s*수\s*없|받아들이지\s*않|'
             r'\bdisagree\w*\b|'
             r'\b(?:do(?:es)?|did|will|would|can|could|ca|wo)\s*n\'?o?t\s+(?:agree|accept)\b|'
             r'\bnot\s+(?:agree|agreed|accepted|acceptable)\b')

# Rule set v2 — (cue, suppress). Same label order as v1.
ACT_RULES_V2 = {
    'claim': (r'맡겠|맡을|맡습니다|가져가겠|가져갈게|가져가도|가져오겠|옮기겠|옮길게|옮깁니다|운반하겠|운반할게|'
              r'운반합니다|배달하겠|배달할게|배달합니다|전달하겠|전달합니다|담당하겠|담당합니다|'
              r'\bclaim(?:s|ing|ed)?\b|\bi will\b|\bi\'?ll\b|\btaking\b|\btakes?\b',
              # someone else's assignment or claim, and requests to take over
              r'\br[123]\b[^.]{0,4}\b(?:takes?|will take|is taking)\b|\bclaimed by\b|'
              r'\b(?:your|his|her|their|its)\s+claim\b|맡아\s?주|맡아\s?달|맡기겠|맡기고'),
    'propose': (r'제안합니다|제안할게|제안해요|제안하겠|제안드립|제안을 올|제안을 제출|제출합니다|제출하겠|'
                r'확정하여 제출|계획을 올|계획을 제출|\bpropos(?:e|es|ing|ed)\b|\bsuggest(?:s|ing|ed)?\b|'
                r'\bproposal:|\bplan:',
                # accepting / awaiting / asking for someone else's proposal is not proposing;
                # confirming or submitting one's OWN proposal is (제출형 propose)
                r'제안을\s*(?:수락|승인|거절|기다)|제안을 제출(?:해|하여|하고)?\s*(?:주|줘)|'
                r'\b(?:accept|accepts|accepted|accepting|reject|rejects|rejected|rejecting|await|awaits|'
                r'awaiting)\b[^.]{0,20}\bproposal\b|'
                r'\bwaiting for\b[^.]{0,20}\bproposal\b'),
    'yield': (r'양보|넘기|넘겨|포기|\byield(?:s|ing|ed)?\b|\bgive\b[^.]{0,24}\bto\b|\bleave\b[^.]{0,24}\bto\b',
              r'양보하지\s*않|포기하지\s*않|\b(?:do(?:es)?|did|will|would|can|could|ca|wo)\s*n\'?o?t\s+'
              r'(?:yield|give|leave)\b'),
    'request': (r'부탁|해 ?주세요|해 ?줘|줘[.!]?$|올려줘|주실|요청|주시겠|주십시오|바랍니다|합시다|하시죠|'
                r'\bplease\b|\bcould you\b|\bcan you\b|\bwould you\b|\blet\'?s\b', ''),
    'agree': (r'좋아|좋습니다|동의합니다|동의해요|동의하며|동의하고|알겠|확인했|그렇게 하|수락합니다|수락할게|'
              r'수락하며|수락하고|\bok\b|\bokay\b|\bagree(?:s|d)?\b|\bsounds good\b|\baccept(?:s|ed|ing)?\b',
              NEG_AGREE + r'|확인하지\s*못|알겠지만|'
              # asking a peer to accept is a request, not the speaker agreeing
              r'\bplease\s+(?:accept|agree)\b|(?:수락|동의)해\s?(?:주세요|주십시오|주시기|줘)'),
    'refuse': (r'거절|거부|반대합니다|안 ?됩니다|불가|못 ?합니다|할 수 없|' + NEG_AGREE +
               r'|\breject\w*\b|\brefuse\w*\b|\bcannot\b|\bcan\'?t\b|\bwon\'?t\b', ''),
    'question': (r'\?|까요|나요|습니까|할래|괜찮을', ''),
    'report': (r'완료|배달했|놓았|도착|끝났|멈췄|중단|\bdelivered\b|\bfinished\b|\bstopped\b|\bdone\b',
               # planned, conditional or negated — not a report of something that happened
               r'(?:완료|도착|중단|배달)(?:하겠|할\s|하면|한 뒤|하는 대로|할 예정|하지\s*(?:못|않)|되지\s*(?:못|않))|'
               r'\b(?:will|would|to|can|could|not)\s+(?:be\s+)?(?:deliver|delivered|finish|finished|stop|'
               r'stopped|done)\b|\bnot\s+(?:yet\s+)?(?:delivered|finished|stopped|done)\b'),
    'inform_obstacle': (r'막혀|막힌|장애물|길을 막|경로가 겹|\bblocked\b|\bobstacle\b|\bin the way\b', ''),
    'correct': (r'정정|수정합니다|수정하겠|아니라|잘못|\bcorrection\b|\bactually\b', ''),
    'standby': (r'대기|기다리|할 일이 없|남은 작업이 없|\bstanding by\b|\bstand by\b|'
                r'\bnothing (?:useful )?remains\b|\bwaiting\b|\bidle\b',
                r'대기하지\s*않|기다리지\s*않'),
}


def _overlaps(span, spans):
    start, end = span
    return any(s < end and start < e for s, e in spans)


def _act_applies(low, cue, suppress):
    """True when at least one cue match is not inside a suppressed span."""
    hits = [m.span() for m in re.finditer(cue, low)]
    if not hits:
        return False
    if not suppress:
        return True
    bad = [m.span() for m in re.finditer(suppress, low)]
    return any(not _overlaps(hit, bad) for hit in hits)


def dialogue_acts(text, rules=ACT_RULES_VERSION):
    """Multi-label dialogue acts of one free-text message. EVALUATION ONLY.

    rules='v2' (default) applies word boundaries and suppression; rules='v1'
    reproduces the frozen rule set that labelled results.v1.
    """
    if rules not in ACT_RULES_VERSIONS:
        raise ValueError(f'rules must be one of {ACT_RULES_VERSIONS}')
    if not text or not text.strip():
        return ['silence']
    low = text.lower()
    if rules == 'v1':
        acts = [act for act, rule in ACT_RULES_V1.items() if re.search(rule, low)]
    else:
        acts = [act for act, (cue, sup) in ACT_RULES_V2.items() if _act_applies(low, cue, sup)]
    return acts or ['other']


def struct_acts(message):
    """V3 structured act. Not comparable to free-text `propose`/`standby` counts."""
    if message is None:
        return ['silence']
    if not isinstance(message, dict):
        return ['invalid']
    act = message.get('act')
    return [act if isinstance(act, str) and act in ACT_LABELS else 'other']


def references(text, peer_utterance, peer_id, labels):
    """Does text mention the peer robot or an item/zone the peer's utterance named?"""
    if not text:
        return False
    if re.search(rf'(?<![A-Za-z0-9]){peer_id}(?![0-9])', text):
        return True
    named = {m for m in re.findall(r'[a-z]+-\d+', peer_utterance or '') if m in labels}
    return any(label in text for label in named)
