from ultralytics import YOLO

# Load YOLOv8 nano
model = YOLO("yolov8n.pt")

# Train
model.train(
    data="data.yaml",
    imgsz=640,
    epochs=100,
    batch=8,
    workers=0,
    device=0   # set to "cpu" if no GPU
)
