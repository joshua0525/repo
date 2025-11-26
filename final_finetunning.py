from ultralytics import YOLO

model = YOLO("yolov8s-pose.pt")
model.train(
    task='pose',
    data=r"C:\yolo_test\care_bot_skelleton.v4i.yolov8\data.yaml",
    epochs=200, imgsz=640, batch=2, workers=0,
    lr0=5e-4, warmup_epochs=5, freeze=10,
    cos_lr=True, amp=False, fliplr=0.0, patience=80, plots=True
)
