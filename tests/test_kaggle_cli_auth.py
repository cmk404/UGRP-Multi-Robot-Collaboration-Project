from datetime import datetime, timedelta, timezone
from scripts.kaggle_cli_auth import refresh_needed


def test_refreshes_before_expiration_not_thirty_minutes_after():
    now=datetime(2026,9,21,4,15,tzinfo=timezone.utc)
    base={'refresh_token':'test-only','access_token':'test-only'}
    for delta,expected in [(-31,True),(-1,True),(0,True),(4,True),(6,False)]:
        assert refresh_needed({**base,'access_token_expiration':(now+timedelta(minutes=delta)).isoformat()},now) is expected
    assert not refresh_needed({},now)
