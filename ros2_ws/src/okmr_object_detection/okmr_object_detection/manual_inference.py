import argparse
import onnxruntime as ort
import numpy as np
import cv2
import os
import torch


def sigmoid(x):
    return 1 / (1 + np.exp(-x))


class YOLOSegONNX:
    def __init__(self, onnx_path, providers=None):
        if not os.path.isfile(onnx_path):
            raise FileNotFoundError(f"ONNX model not found: {onnx_path}")
        self.session = ort.InferenceSession(
            onnx_path,
            providers=[
                "TensorrtExecutionProvider",
                "CUDAExecutionProvider",
                "CPUExecutionProvider",
            ],
        )

        # Inspect output names & shapes
        outs = self.session.get_outputs()
        names_shapes = [(o.name, o.shape) for o in outs]
        print("== ONNX outputs ==")
        for n, s in names_shapes:
            print(f"  {n:<30} shape={s}")

        # Single-input name
        self.input_name = self.session.get_inputs()[0].name

        # Branch: 3-output seg vs 2-output raw+proto
        if len(outs) == 3:
            # standard seg export: [det, coefs, proto]
            det3, coef3, proto3 = outs
            self.det_name = det3.name
            self.coef_name = coef3.name
            self.proto_name = proto3.name
            self.mode_2out = False

        elif len(outs) == 2:
            # raw export: [raw_preds, proto]
            raw, proto = outs
            self.raw_name = raw.name
            self.proto_name = proto.name
            self.det_name = None
            self.coef_name = None
            self.mode_2out = True

        else:
            raise ValueError(f"Expected 2 or 3 outputs, got: {names_shapes}")

    def preprocess(self, img_path, input_size=640):
        img0 = cv2.imread(img_path)
        if img0 is None:
            raise ValueError(f"Could not load image: {img_path}")
        h0, w0 = img0.shape[:2]
        scale = input_size / max(h0, w0)
        nw, nh = int(w0 * scale), int(h0 * scale)
        img = cv2.resize(img0, (nw, nh), interpolation=cv2.INTER_LINEAR)

        pad_w, pad_h = input_size - nw, input_size - nh
        top, bot = pad_h // 2, pad_h - pad_h // 2
        left, right = pad_w // 2, pad_w - pad_w // 2
        img = cv2.copyMakeBorder(
            img, top, bot, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114)
        )

        # BGR→RGB, HWC→CHW, normalize
        img = img[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
        return np.expand_dims(img, 0), img0, scale, left, top

    def postprocess(
        self,
        det,
        coefs,
        proto,
        orig_img,
        scale,
        pad_x,
        pad_y,
        conf_thresh=0.25,
        mask_thresh=0.5,
        debug=False,
        top_k=None,
    ):

        # det:   (1, N, 6) → [x1,y1,x2,y2,conf,cls]
        # coefs: (1, N, M)
        # proto: (1, M, Hp, Wp)
        det = det[0]
        coefs = coefs[0]
        proto = proto[0]  # (M, Hp, Wp)

        if debug:
            print(f"DEBUG: det shape: {det.shape}")
            print(f"DEBUG: coefs shape: {coefs.shape}")
            print(f"DEBUG: proto shape: {proto.shape}")
            print(
                f"DEBUG: confidence scores range: {det[:, 4].min():.4f} - {det[:, 4].max():.4f}"
            )
            print(f"DEBUG: total detections before filtering: {len(det)}")

        # filter by conf
        scores = det[:, 4]
        keep = scores > conf_thresh
        det_filtered = det[keep]
        coefs_filtered = coefs[keep]

        if debug:
            print(
                f"DEBUG: detections after confidence filtering ({conf_thresh}): {len(det_filtered)}"
            )
            if len(det_filtered) > 0:
                print(
                    f"DEBUG: top 5 confidence scores: {sorted(det_filtered[:, 4], reverse=True)[:5]}"
                )
                print(
                    f"DEBUG: class IDs found: {np.unique(det_filtered[:, 5].astype(int))}"
                )

        # Apply top-k filtering if specified
        if top_k is not None and len(det_filtered) > top_k:
            # Sort by confidence (descending) and keep top k
            conf_indices = np.argsort(det_filtered[:, 4])[::-1][:top_k]
            det_filtered = det_filtered[conf_indices]
            coefs_filtered = coefs_filtered[conf_indices]

            if debug:
                print(f"DEBUG: Keeping only top {top_k} most confident detections")
                print(f"DEBUG: Top {top_k} confidence scores: {det_filtered[:, 4]}")

        # If no detections, return original image with debug info
        if len(det_filtered) == 0:
            if debug:
                print("DEBUG: No detections found! Returning original image.")
                # Show top 10 confidence scores even below threshold
                top_indices = np.argsort(scores)[-10:][::-1]
                print("DEBUG: Top 10 confidence scores (even below threshold):")
                for i, idx in enumerate(top_indices):
                    print(f"  {i+1}: conf={scores[idx]:.4f}, class={int(det[idx, 5])}")
            return orig_img

        # decode masks
        M, Hp, Wp = proto.shape
        proto_flat = proto.reshape(M, -1)  # (M, Hp*Wp)
        masks = coefs_filtered @ proto_flat  # (K, Hp*Wp)
        masks = masks.reshape(-1, Hp, Wp)  # (K, Hp, Wp)

        out = orig_img.copy()
        H0, W0 = orig_img.shape[:2]

        for i, d in enumerate(det_filtered):
            x1, y1, x2, y2, conf, cls = d
            # undo letterbox & scale
            x1 = int((x1 - pad_x) / scale)
            y1 = int((y1 - pad_y) / scale)
            x2 = int((x2 - pad_x) / scale)
            y2 = int((y2 - pad_y) / scale)

            # draw bbox & label
            label = f"{int(cls)} {conf:.2f}"
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                out, label, (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
            )

            # # mask
            # mask_logit = masks[i]
            # mask = sigmoid(mask_logit)
            # mask = cv2.resize(mask, (W0, int(H0)))
            # bin_mask = (mask > mask_thresh).astype(np.uint8) * 255
            # mask
            mask = sigmoid(masks[i])
            # proto is in letterboxed input space (stride 4): upsample to input as shown online
            # size, strip the letterbox padding, then resize to original image
            in_w, in_h = Wp * 4, Hp * 4
            mask = cv2.resize(mask, (in_w, in_h))
            top, left = int(round(pad_y)), int(round(pad_x))
            bottom = int(round(pad_y + H0 * scale))
            right = int(round(pad_x + W0 * scale))
            mask = mask[top:bottom, left:right]
            mask = cv2.resize(mask, (W0, H0))

            # keep only the pixels inside this detection's box
            bx1, by1 = max(x1, 0), max(y1, 0)
            bx2, by2 = min(x2, W0), min(y2, H0)
            box_mask = np.zeros_like(mask)
            box_mask[by1:by2, bx1:bx2] = mask[by1:by2, bx1:bx2]
            bin_mask = (box_mask > mask_thresh).astype(np.uint8) * 255

            # overlay green
            colored = np.zeros_like(out)
            colored[:, :, 1] = bin_mask
            out = cv2.addWeighted(out, 1.0, colored, 0.5, 0)

            if debug:
                print(
                    f"DEBUG: Detection {i+1}: bbox=({x1},{y1},{x2},{y2}), conf={conf:.4f}, class={int(cls)}"
                )

        return out


    def infer(
        self,
        img_path,
        img_size=640,
        conf_thresh=0.25,
        mask_thresh=0.5,
        show=True,
        save_path=None,
        debug=False,
        top_k=None,
    ):

        inp, orig, scale, pad_x, pad_y = self.preprocess(img_path, img_size)

        if debug:
            print(f"DEBUG: Input shape: {inp.shape}")
            print(f"DEBUG: Original image shape: {orig.shape}")
            print(f"DEBUG: Scale factor: {scale}")
            print(f"DEBUG: Padding: x={pad_x}, y={pad_y}")

        outs = self.session.run(None, {self.input_name: inp})

        if self.mode_2out:
            # TWO-output model: outs[0]=raw_preds (1,C,N), outs[1]=proto
            raw_preds, proto = outs
            _, C, N = raw_preds.shape
            M = proto.shape[1]

            if debug:
                print(f"DEBUG: Raw predictions shape: {raw_preds.shape}")
                print(f"DEBUG: Proto shape: {proto.shape}")
                print(f"DEBUG: C={C}, N={N}, M={M}")

            # transpose raw → (1,N,C)
            raw = raw_preds.transpose(0, 2, 1)

            # split channels - fix the channel calculation
            xywh = raw[..., :4]
            obj_conf = raw[..., 4:5]

            # Calculate number of classes dynamically
            num_classes = (
                C - 5 - M
            )  # Total channels - bbox(4) - objectness(1) - mask_coeffs(M)

            if debug:
                print(f"DEBUG: Calculated number of classes: {num_classes}")

            # Handle case where num_classes <= 0
            if num_classes <= 0:
                print(
                    f"WARNING: Calculated num_classes={num_classes}, treating as single class model"
                )
                # For single class models, use objectness as class score
                cls_scores = obj_conf  # Use objectness as class confidence
                cls_id = np.zeros_like(obj_conf)  # Class 0 for all detections
                mask_coefs = raw[..., 5:]  # Rest are mask coefficients
            else:
                cls_scores = raw[..., 5 : 5 + num_classes]
                mask_coefs = raw[..., 5 + num_classes :]

                if debug:
                    print(f"DEBUG: cls_scores shape: {cls_scores.shape}")
                    print(f"DEBUG: mask_coefs shape: {mask_coefs.shape}")
                    if cls_scores.size > 0:
                        print(
                            f"DEBUG: cls_scores range: {cls_scores.min():.4f} - {cls_scores.max():.4f}"
                        )

                # Get best class
                cls_id = np.expand_dims(cls_scores.argmax(-1), axis=-1)
                cls_conf = np.expand_dims(cls_scores.max(-1), axis=-1)
                # Use combined confidence
                obj_conf = obj_conf * cls_conf

            if debug:
                print(
                    f"DEBUG: obj_conf range: {obj_conf.min():.4f} - {obj_conf.max():.4f}"
                )

            # decode xywh - keep same dimensions by using expand_dims
            xc, yc, w, h = (
                xywh[..., 0:1],
                xywh[..., 1:2],
                xywh[..., 2:3],
                xywh[..., 3:4],
            )
            x1 = xc - w / 2
            y1 = yc - h / 2
            x2 = xc + w / 2
            y2 = yc + h / 2

            # det (1,N,6) - all arrays now have same dimensions
            if num_classes <= 0:
                det = np.concatenate([x1, y1, x2, y2, obj_conf, cls_id], axis=-1)
            else:
                det = np.concatenate([x1, y1, x2, y2, obj_conf, cls_id], axis=-1)

            coefs = mask_coefs
        else:
            # THREE-output model
            det, coefs, proto = outs[0], outs[1], outs[2]

        result = self.postprocess(
            det,
            coefs,
            proto,
            orig,
            scale,
            pad_x,
            pad_y,
            conf_thresh,
            mask_thresh,
            debug=debug,
            top_k=top_k,
        )

        if save_path:
            cv2.imwrite(save_path, result)
            print(f"Saved: {save_path}")
        if show:
            cv2.imshow("YOLOv11‑Seg ONNX", result)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        default="model.onnx",
        required=False,
        help="Path to yolov11n-seg.onnx",
    )
    parser.add_argument(
        "--image",
        default="image.png",
        required=False,
        help="Input image path",
    )
    parser.add_argument("--img-size", type=int, default=640)
    parser.add_argument("--conf-thresh", type=float, default=0.5)
    parser.add_argument("--mask-thresh", type=float, default=0.5)
    parser.add_argument(
        "--top-k",
        type=int,
        default=None,
        help="Show only top K most confident detections (e.g., 5)",
    )
    parser.add_argument("--no-show", action="store_true")
    parser.add_argument("--save", help="Optional output path")
    parser.add_argument("--debug", action="store_true", help="Enable debug output")
    args = parser.parse_args()

    inferer = YOLOSegONNX(args.model)
    inferer.infer(
        img_path=args.image,
        img_size=args.img_size,
        conf_thresh=args.conf_thresh,
        mask_thresh=args.mask_thresh,
        show=not args.no_show,
        save_path=args.save,
        debug=args.debug,
        top_k=args.top_k,
    )
