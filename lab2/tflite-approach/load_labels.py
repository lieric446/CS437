def load_labels(path):
    with open(path, "r", encoding="utf-8") as f:
        lines = [ln.strip() for ln in f.readlines() if ln.strip()]
    labels = {}
    for i, line in enumerate(lines):
        parts = line.split(maxsplit=1)
        if len(parts) == 2 and parts[0].isdigit():
            labels[int(parts[0])] = parts[1]
        else:
            labels[i] = line
    return labels
