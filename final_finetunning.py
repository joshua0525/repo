from ultralytics import YOLO

model = YOLO("yolov8s-pose.pt")
model.train(
    task='pose',
    data=r"C:\yolo_test\care_bot_skelleton.v4i.yolov8\data.yaml",
    epochs=200, imgsz=640, batch=2, workers=0,
    lr0=5e-4, warmup_epochs=5, freeze=10,
    cos_lr=True, amp=False, fliplr=0.0, patience=80, plots=True
)



"""
model = YOLO(r"D:\capstone\carebot\pi_program\runs\pose\train13\weights\best.pt")
model.train(
    task='pose',
    data=r"C:\yolo_test\care_bot_skelleton.v3i.yolov8\data_pose.yaml",
    epochs=120, imgsz=640, batch=2, workers=0,
    lr0=3e-4, warmup_epochs=3, freeze=0,
    cos_lr=True, amp=False, fliplr=0.5, patience=60, plots=True
)

# 3) (선택) 플립 ON으로 파인튜닝 — flip_idx 정상 확인 후에만 실행
# best = r"runs\pose\trainXX\weights\best.pt"  # 위 학습 결과 경로로 교체
# model = YOLO(best)
# model.train(
#     task='pose',
#     data=DATA,
#     epochs=100, imgsz=640, batch=2, workers=0,
#     lr0=3e-4, warmup_epochs=3, freeze=0,
#     cos_lr=True, amp=False, fliplr=0.5, patience=50,
#     plots=True
# )
"""