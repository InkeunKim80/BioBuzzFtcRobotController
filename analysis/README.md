# RPM Control PID Analysis

PID 튜닝을 위한 자동 분석 및 추천 도구

## 📋 사용 방법

### 1. 로그 데이터 수집

**ShootTestDualMotor.java** 또는 **ShootTestSingleMotor.java**에서:

```java
private RpmControlLogger logger;

@Override
public void start() {
    // ... 기존 코드 ...
    logger = new RpmControlLogger("ShootTestDualMotor");
    telemetry.addData("Log Path", logger.filePath());
}

@Override
public void periodic() {
    shooterDual.periodic();
    
    // 로깅 (메커니즘에서 getter 필요)
    logger.log(
        shooterDual.getTargetRPM(),
        shooterDual.getRPM1(),
        shooterDual.getRPM2(),
        // error, power, etc...
        shooterDual.getState().toString()
    );
}

@Override
public void end() {
    logger.close();
}
```

### 2. 로그 파일 다운로드

Control Hub에서:
- **웹 파일매니저**: Program & Manage → Manage → Files → `/sdcard/FIRST/rpm_logs/`
- **ADB**: `adb pull /sdcard/FIRST/rpm_logs/ShootTestDualMotor_*.csv`

### 3. Jupyter Notebook 실행

```bash
cd analysis
jupyter notebook rpm_control_analysis.ipynb
```

### 4. CSV 파일 지정

Notebook 첫 셀에서:
```python
log_file = 'path/to/your/ShootTestDualMotor_20261006_120000.csv'
```

### 5. 분석 실행

모든 셀을 실행하면:
- 📊 **6개의 분석 그래프** 자동 생성
- 📈 **시스템 응답 특성** 계산
- 💡 **권장 P, I, D 값** 자동 추천
- 💾 **코드 직접 복사 가능한 형식** 제공

---

## 📊 생성되는 그래프

1. **Target vs Actual RPM** - 모터가 목표에 얼마나 빨리 도달하는가?
2. **RPM Error** - 시간에 따른 에러 변화
3. **Power Output** - 스로틀 값의 변화
4. **PID Output vs Power** - 계산된 값 vs 최종 출력
5. **Integral Error** - I term이 누적되는 방식
6. **Phase Space Plot** - 에러와 에러 변화율의 관계 (진동 판단)

---

## 💡 자동 진단 항목

```
✓ 오버슈트 감지
✓ 진동 분석
✓ 정상상태 에러 측정
✓ 상승 시간 계산
✓ 안정화 시간 계산
```

---

## 🎯 추천 값 자동 계산

기준:
- **P 조정**: 응답 속도 vs 오버슈트
- **I 조정**: 정상상태 에러 감소
- **D 조정**: 진동 감쇠

결과 예시:
```
📈 RECOMMENDED PID VALUES:
   P = 0.001500 (기존: 0.001000)
   I = 0.000400 (기존: 0.000200)
   D = 0.001000 (기존: 0.000000)

💾 UPDATE YOUR CODE:
   private static final double KP = 0.001500;
   private static final double KI = 0.000400;
   private static final double KD = 0.001000;
```

---

## 📝 튜닝 워크플로우

1. **첫 번째 테스트**: 기본 P, I, D로 로그 수집
2. **분석 실행**: 추천 값 확인
3. **값 업데이트**: 코드에 적용
4. **재테스트**: 새 로그 수집
5. **반복**: 만족할 때까지 반복

---

## 🔧 트러블슈팅

**로그 파일을 찾을 수 없음**
- → 로그 경로를 명시적으로 지정하세요: `log_file = '/path/to/file.csv'`

**그래프가 이상하게 표시됨**
- → CSV 파일이 제대로 생성되었는지 확인
- → 모터 전원이 켜져 있었는지 확인

**추천 값이 이상함**
- → 테스트 시간이 너무 짧을 수 있음 (최소 10초 권장)
- → 타겟 RPM이 0에 너무 가까우면 부정확함

---

## 📦 필수 라이브러리

```bash
pip install pandas numpy matplotlib
```

---

## 💾 출력 파일

- `rpm_control_analysis.png` - 분석 그래프 (저해상도)
- `pid_tuning_summary.txt` - 텍스트 요약

---

**Happy Tuning! 🚀**
