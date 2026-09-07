# MasterPi Control frontend

`dashboard/server.py`가 직접 제공하는 빌드 없는 정적 UI입니다.

```bash
scripts/start_masterpi_dashboard.sh --transport ssh
```

`http://127.0.0.1:8765`에서 엽니다. `/api/status`가 SSH와 I2C 컨트롤러를 모두 확인해야 동작 버튼이 활성화됩니다.
