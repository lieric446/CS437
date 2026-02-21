from tflite_runtime.interpreter import Interpreter
from typing import Dict, List, Tuple, Optional
import numpy as np


def looks_like_scores(arr: np.ndarray) -> bool:
    a = arr.astype(np.float32).flatten()
    return np.all((a >= -0.01) & (a <= 1.01))

def parse_ssd(interpreter: Interpreter, labels: Dict[int,str], threshold: float, topk: int) -> List[Tuple[str, float]]:
    """
    Parses SSD MobileNet outputs:
      boxes   [1, N, 4]
      classes [1, N]
      scores  [1, N]
      count   [1]
    Returns: [(label, score), ...]
    """
    out_details = interpreter.get_output_details()
    outs = [np.array(interpreter.get_tensor(d["index"])) for d in out_details]

    boxes = None
    one_by_n = []
    count = None

    for o in outs:
        if o.ndim == 3 and o.shape[0] == 1 and o.shape[2] == 4:
            boxes = o
        elif o.ndim == 2 and o.shape[0] == 1:
            one_by_n.append(o)
        elif o.ndim == 1 and o.shape[0] == 1:
            count = int(o[0])

    if boxes is None or len(one_by_n) < 2:
        return []

    a, b = one_by_n[0], one_by_n[1]
    if looks_like_scores(a) and not looks_like_scores(b):
        scores, classes = a, b
    elif looks_like_scores(b) and not looks_like_scores(a):
        scores, classes = b, a
    else:
        # fallback: most common ordering in many exports is classes then scores
        classes, scores = a, b

    if count is None:
        count = boxes.shape[1]

    cls = classes.flatten()
    scr = scores.flatten().astype(np.float32)

    n = min(count, len(cls), len(scr))
    dets: List[Tuple[str, float]] = []
    for i in range(n):
        s = float(scr[i])
        if s < threshold:
            continue
        cid = int(cls[i])
        name = labels.get(cid, f"class_{cid}")
        dets.append((name, s))

    dets.sort(key=lambda x: x[1], reverse=True)
    return dets[:topk]
