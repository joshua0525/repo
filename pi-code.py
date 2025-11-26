import os, time, sys
import cv2
import numpy as np
from ultralytics import YOLO
from gpiozero import DistanceSensor 

# ---------------------- [설정값 수정 구역] ----------------------
# 1. 카메라 설정
CAM_ID          = 0
CAP_WIDTH       = 640
CAP_HEIGHT      = 480
SHOW_WINDOW     = False # 화면 없이 실행 (Headless 모드)

# 2. 모델 설정
MODEL_PATH      = "best_final.pt"  # 학습된 모델 경로
IMGSZ           = 416
CONF_THRESHOLD  = 0.50
DEVICE          = "cpu"

# 3. 클래스 ID (★data.yaml 파일 확인 필수★)
# 예: ['chest', 'head', 'hip', 'thigh'] 순서라면 chest=0, hip=2
TARGET_CHEST_CLASS = 0 
TARGET_HIPS_CLASS  = 2 

# 4. 제어 파라미터
CENTER_DEADBAND_PX = 40 # 중앙 정렬 오차 허용 범위 (픽셀)
LIFT_DISTANCE_CM   = 6  # 이 거리 안으로 들어오면 '가깝다'고 판단

# 5. 하드웨어(초음파/시리얼) 설정
GPIO_TRIGGER    = 23
GPIO_ECHO       = 24
SERIAL_PORT     = "/dev/ttyACM0"
BAUD            = 115200 # 아두이노와 속도 통일

# 6. 성능 설정
SKIP_N          = 1      # 프레임 스킵 (1이면 2번 중 1번만 추론)
# --------------------------------------------------------------

# --- 시리얼 통신 초기화 ---
ser = None
try:
    import serial
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.001)
    time.sleep(2.0) # 아두이노 리셋 대기
    print(f"[INFO] Serial connected: {SERIAL_PORT} @ {BAUD}")
except Exception as e:
    print(f"[WARN] Serial open failed: {e}")
    ser = None

def send_to_arduino(command: str):
    """ 명령어가 바뀔 때만 아두이노로 전송 (중복 전송 방지) """
    if command is None: return
    
    # 이전 명령과 같으면 전송하지 않음 (단, 최초 실행 시는 전송)
    if command == send_to_arduino.last_cmd:
        return

    if ser is not None and ser.writable():
        try:
            ser.write((command + "\n").encode("utf-8"))
            send_to_arduino.last_cmd = command
            print(f"[TX] >>> {command}")
        except Exception as e:
            print(f"[WARN] Serial TX error: {e}")
    else:
        # 시리얼 연결 안 됨 (테스트용)
        if command != send_to_arduino.last_cmd:
            print(f"[TX-DRY] >>> {command}")
            send_to_arduino.last_cmd = command

send_to_arduino.last_cmd = None

# --- 초음파 센서 초기화 ---
try:
    sensor = DistanceSensor(echo=GPIO_ECHO, trigger=GPIO_TRIGGER)
    print("[INFO] Ultrasonic sensor initialized.")
except Exception as e:
    print(f"[ERROR] GPIO Init failed: {e}")
    sys.exit(1)

def get_distance_cm():
    """ 거리 측정 (센서가 막히거나 튀면 None 반환) """
    try:
        # gpiozero는 거리를 미터 단위로 반환 (최대 1m 기본값)
        dist = sensor.distance * 100 
        # 값이 너무 크거나(센서 튐) 잡히지 않으면 None 처리
        if dist > 1000: return None 
        return dist
    except:
        return None

def parse_detections(result):
    """ YOLO 결과에서 Chest와 Hips 박스 추출 """
    if result is None or len(result.boxes) == 0:
        return None, None

    boxes = result.boxes.cpu().numpy()
    best_chest = None
    best_hips = None
    max_chest_conf = 0.0
    max_hips_conf = 0.0

    for i in range(len(boxes.cls)):
        cls_id = int(boxes.cls[i])
        conf = boxes.conf[i]
        
        if cls_id == TARGET_CHEST_CLASS and conf > max_chest_conf:
            best_chest = boxes.xyxy[i]
            max_chest_conf = conf
        elif cls_id == TARGET_HIPS_CLASS and conf > max_hips_conf:
            best_hips = boxes.xyxy[i]
            max_hips_conf = conf

    return best_chest, best_hips

def decide_command(frame_w, frame_h, chest_box, hips_box, current_cm, is_attached):
    """ 
    [상태 머신 로직]
    is_attached: 현재 로봇이 물체를 잡고 있는지 여부 (True/False)
    return: (명령어 String, 새로운 is_attached 상태)
    """
    
    # -------------------------------------------------
    # CASE 1: 이미 물체를 잡고 있는 상태 (부착됨)
    # -------------------------------------------------
    if is_attached:
        # 센서가 완전히 밀착되면 초음파는 거리를 못 재거나(None), 
        # 혹은 매우 큰 노이즈 값으로 튈 수 있음. 이를 '밀착'으로 판단.
        if current_cm is None:
            print("[LOG] Sensor blocked (Attached). Dropping (PD).")
            return "PD", False # 내려놓기 명령 후 -> 상태 해제
        else:
            # 아직 거리가 측정됨 = 잡고 이동 중이거나 대기 중
            return "S", True # 정지 상태 유지, 부착 상태 유지

    # -------------------------------------------------
    # CASE 2: 물체를 찾아가는 상태 (탐색/접근)
    # -------------------------------------------------
    
    # 1. 정렬 기준(Chest) 확인
    if chest_box is None:
        return "S", False # 안 보이면 정지

    # Chest의 중심 좌표 계산
    x1, _, x2, _ = chest_box
    chest_cx = (x1 + x2) / 2.0
    screen_cx = frame_w / 2.0
    dx = chest_cx - screen_cx

    # 2. 좌우(L/R) 정렬 우선
    if dx > CENTER_DEADBAND_PX:
        return "R", False
    elif dx < -CENTER_DEADBAND_PX:
        return "L", False

    # 3. 거리(F) 좁히기
    # 센서 값이 없으면(None) 위험하므로 일단 정지
    if current_cm is None:
        return "S", False 

    if current_cm > LIFT_DISTANCE_CM:
        return "F", False # 아직 멈 (전진)

    # 4. 최종 짚기(PU) 판단
    # 거리가 좁혀졌고(6cm 이내), 엉덩이(Hips)까지 식별되면 집어올림
    if hips_box is not None:
        print("[LOG] Target aligned & Close. Picking Up (PU).")
        return "PU", True # 집기 명령 후 -> 부착 상태로 변경
    else:
        # 거리는 맞는데 엉덩이가 가려졌거나 인식이 안 됨 -> 대기
        return "S", False

def main():
    # 1. 모델 로드
    print(f"[INFO] Loading YOLO model: {MODEL_PATH}...")
    model = YOLO(MODEL_PATH)

    # 2. 카메라 오픈
    cap = cv2.VideoCapture(CAM_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_HEIGHT)
    if not cap.isOpened():
        print("[ERROR] Camera open failed")
        sys.exit(1)

    print("[INFO] System Ready. Headless Mode.")
    
    # 추론 결과 저장용 변수 (프레임 스킵 시 사용)
    last_chest = None
    last_hips = None
    
    # ★ 상태 변수: 로봇이 물체를 잡았는가?
    is_attached = False 
    
    frame_count = 0

    while True:
        try:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue

            # 1. 거리 측정 (매 프레임 수행 - 안전을 위해)
            dist = get_distance_cm()

            # 2. YOLO 추론 (SKIP_N에 따라 간헐적 수행)
            do_infer = (frame_count % (SKIP_N + 1) == 0)
            frame_count += 1

            if do_infer:
                results = model(frame, device=DEVICE, verbose=False, conf=CONF_THRESHOLD, imgsz=IMGSZ)
                if len(results) > 0:
                    last_chest, last_hips = parse_detections(results[0])
                else:
                    last_chest, last_hips = None, None
            
            # 3. 명령 결정 (매 프레임 수행)
            h, w = frame.shape[:2]
            cmd, is_attached = decide_command(w, h, last_chest, last_hips, dist, is_attached)

            # 4. 아두이노 전송
            send_to_arduino(cmd)
            
            # Headless 모드이므로 imshow 없음

        except KeyboardInterrupt:
            print("\n[INFO] Stopping...")
            send_to_arduino("S")
            break
        except Exception as e:
            print(f"[ERROR] Loop error: {e}")
            send_to_arduino("S")
            time.sleep(1)

    cap.release()
    if ser: ser.close()
    sensor.close()
    print("[INFO] Program Finished.")

if __name__ == "__main__":
    main()