from pathlib import Path

IMAGES_DIR = Path("dataset/images")
LABELS_DIR = Path("dataset/labels")

for split in ["train", "val"]:
    images = list((IMAGES_DIR / split).glob("*.jpg"))
    labels = list((LABELS_DIR / split).glob("*.txt"))

    for img in images:
        matches = [l for l in labels if img.stem in l.stem]

        if len(matches) != 1:
            print(f"❌ {img.name}: found {len(matches)} labels")
            continue

        old_label = matches[0]
        new_label = LABELS_DIR / split / f"{img.stem}.txt"

        old_label.rename(new_label)
        print(f"✅ {old_label.name} → {new_label.name}")
