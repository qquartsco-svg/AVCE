# AVCE 블록체인 서명

핵심 모듈 파일의 **SHA256 해시**를 기록해 변조 추적을 지원합니다.

## 서명 생성

```bash
# avce 루트에서
python3 blockchain/sign_avce.py
```

→ `blockchain/signature.json`, `blockchain/SIGNATURE.md` 생성.

## 검증

- `signature.json`: 파일별 sha256, signed_at, bytes.
- 동일 디렉터리에서 다시 `sign_avce.py` 실행 후 기존 `signature.json`과 비교하면 변경 여부 확인 가능.

## PHAM v4 연동 (선택)

CookiieBrain 프로젝트의 `blockchain/pham_sign_v4.py`로 개별 파일을 서명하면 `pham_chain_*.json` 체인이 생성됩니다. AVCE 단독 배포 시에는 위 `sign_avce.py`만으로도 무결성 기록이 가능합니다.
