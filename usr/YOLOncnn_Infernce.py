from ultralytics import YOLO

# Load the exported NCNN model
ncnn_model = YOLO("../NN_model/yolo11n_ncnn_model",task="detect");

# Run inference
results = ncnn_model("../NN_model/assets/bus.jpg")

#check the results
print(results[0].boxes)