from roboflow import Roboflow

rf = Roboflow(api_key="9mSngBAfmaGYnUfVWAXh")
project = rf.workspace("week4-kqz2h").project("week4-5jp5x")
version = project.version(2)
#dataset = version.download("coco")
model = project.version(2).model
model.download(["YOLOv11.pt"])