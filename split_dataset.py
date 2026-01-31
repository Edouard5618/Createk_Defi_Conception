import random
import shutil
from pathlib import Path

# ===== CONFIG =====
DATASET_ROOT = Path("dataset")
IMAGES_SRC = DATASET_ROOT / "images"
LABELS_SRC = DATASET_ROOT / "labels"

TRAIN_COUNT = 49
VAL_COUNT = 20
RANDOM_SEED = 42
# ==================

random.seed(RANDOM_SEED)

# Create output folders
for split in ["train", "val"]:
    (IMAGES_SRC / split).mkdir(parents=True, exist_ok=True)
    (LABELS_SRC / split).mkdir(parents=True, exist_ok=True)

# Collect images (not already split)
images = [
    p for p in IMAGES_SRC.glob("*.jpg")
]
print(len(images))
assert len(images) >= TRAIN_COUNT + VAL_COUNT, "Not enough images"

random.shuffle(images)

train_images = images[:TRAIN_COUNT]
val_images = images[TRAIN_COUNT:TRAIN_COUNT + VAL_COUNT]


def find_label_for_image(image_path: Path) -> Path:
    """
    Label files contain the image filename inside their name.
    Example:
    image: WIN_20260131_10_54_45_Pro.jpg
    label: xxxx-WIN_20260131_10_54_45_Pro.txt
    """
    matches = list(LABELS_SRC.glob(f"*{image_path.stem}*.txt"))
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected 1 label for {image_path.name}, found {len(matches)}"
        )
    return matches[0]


def move_split(image_list, split_name):
    for img in image_list:
        lbl = find_label_for_image(img)

        shutil.move(
            str(img),
            IMAGES_SRC / split_name / img.name
        )
        shutil.move(
            str(lbl),
            LABELS_SRC / split_name / lbl.name
        )


move_split(train_images, "train")
move_split(val_images, "val")

print("✅ Dataset split complete")
print(f"Train: {len(train_images)}")
print(f"Val:   {len(val_images)}")
