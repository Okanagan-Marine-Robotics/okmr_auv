import onnxruntime as ort
import okmr_object_detection.detector
from okmr_object_detection.detector import ObjectDetectorNode
import rclpy
import numpy as np
import cv2
import os
import threading
from typing import Tuple, Optional
from okmr_msgs.srv import ChangeModel, SetInferenceCamera


def sigmoid(x):
    return 1 / (1 + np.exp(-x))


# Reason why file is so long is from pre+post process methods i found to be necessary when testing outside of ros2


class OnnxSegmentationDetector(ObjectDetectorNode):
    def __init__(self):
        super().__init__(node_name="onnx_segmentation_detector")

        self.declare_parameter("model_path", "gate.onnx")
        self.declare_parameter("conf_threshold", 0.4)
        self.declare_parameter("mask_threshold", 0.3)
        self.declare_parameter("input_size", 640)
        self.declare_parameter("top_k", 5)  # -1 means no limit
        self.declare_parameter(
            "providers",
            [
                # "TensorrtExecutionProvider",
                "CUDAExecutionProvider",
                # "CPUExecutionProvider",
            ],
        )
        self.declare_parameter("debug", True)

        # Get parameters
        model_filename = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        
        # Check if it's a full path or just filename
        if os.path.isabs(model_filename):
            self.model_path = model_filename
        else:
            # Use the installed models directory
            from ament_index_python.packages import get_package_share_directory
            package_share = get_package_share_directory('okmr_object_detection')
            self.model_path = os.path.join(package_share, 'models', model_filename)
        self.conf_threshold = (
            self.get_parameter("conf_threshold").get_parameter_value().double_value
        )
        self.mask_threshold = (
            self.get_parameter("mask_threshold").get_parameter_value().double_value
        )
        self.input_size = (
            self.get_parameter("input_size").get_parameter_value().integer_value
        )
        self.top_k = self.get_parameter("top_k").get_parameter_value().integer_value
        self.providers = (
            self.get_parameter("providers").get_parameter_value().string_array_value
        )
        self.debug = self.get_parameter("debug").get_parameter_value().bool_value

        if self.top_k == -1:
            self.top_k = None

        # Model mapping
        self.model_mapping = {
            ChangeModel.Request.GATE: "gate.onnx",
            ChangeModel.Request.SHARK: "shark.onnx",
            ChangeModel.Request.SWORDFISH: "swordfish.onnx",
            ChangeModel.Request.PATH_MARKER: "path_marker.onnx",
            ChangeModel.Request.SLALOM_CENTER: "slalom_center.onnx",
            ChangeModel.Request.SLALOM_OUTER: "slalom_outer.onnx",
            ChangeModel.Request.DROPPER_BIN: "dropper_bin.onnx",
            ChangeModel.Request.TORPEDO_BOARD: "torpedo_board.onnx"
        }

        # Create services
        self.change_model_srv = self.create_service(
            ChangeModel,
            '/change_model',
            self.change_model_callback
        )

        self.set_inference_camera_srv = self.create_service(
            SetInferenceCamera,
            '/set_inference_camera',
            self.set_inference_camera_callback
        )
        
        # Camera state tracking
        self.camera_mode = SetInferenceCamera.Request.FRONT_CAMERA  # Default to front camera
        self.inference_enabled = True

        
        self.model_lock = threading.Lock()

        self.load_model()

        self.get_logger().info(f"providers: {self.providers}")
        self.get_logger().info(f"ONNX Segmentation Detector initialized")
        self.get_logger().info(f"Model: {self.model_path}")
        self.get_logger().info(f"Confidence threshold: {self.conf_threshold}")
        self.get_logger().info(f"Input size: {self.input_size}")

    def __del__(self):
        """Destructor to clean up OpenCV windows."""
        if self.debug:
            cv2.destroyAllWindows()

    def change_model_callback(self, request, response):
        """Service callback to change the model file"""
        with self.model_lock:
            try:
                if request.model_id not in self.model_mapping:
                    response.success = False
                    response.message = f"Invalid model ID: {request.model_id}. Valid IDs are: {list(self.model_mapping.keys())}"
                    return response

                # Get the model filename
                model_filename = self.model_mapping[request.model_id]
                
                # Construct the full path to the installed models directory
                from ament_index_python.packages import get_package_share_directory
                package_share = get_package_share_directory('okmr_object_detection')
                new_model_path = os.path.join(package_share, 'models', model_filename)
                
                # Check if the model file exists
                if not os.path.isfile(new_model_path):
                    response.success = False
                    response.message = f"Model file not found: {new_model_path}"
                    return response

                # Update the model path and reload
                old_model = self.model_path
                self.model_path = new_model_path
                
                try:
                    self.load_model()
                    response.success = True
                    response.message = f"Successfully changed model from {os.path.basename(old_model)} to {model_filename}"
                    self.get_logger().info(f"Model changed to: {self.model_path}")
                    
                except Exception as e:
                    # Revert to old model if loading fails
                    self.model_path = old_model
                    self.load_model()
                    response.success = False
                    response.message = f"Failed to load new model, reverted to previous: {str(e)}"
                    
            except Exception as e:
                response.success = False
                response.message = f"Service error: {str(e)}"
            
        return response

    def set_inference_camera_callback(self, request, response):
        """Service callback to set inference camera mode"""
        try:
            if request.camera_mode == SetInferenceCamera.Request.DISABLED:
                self.inference_enabled = False
                self.camera_mode = request.camera_mode
                response.success = True
                response.message = "Inference disabled"
                self.get_logger().info("Inference disabled")
                
            elif request.camera_mode == SetInferenceCamera.Request.FRONT_CAMERA:
                self.inference_enabled = True
                self.camera_mode = request.camera_mode
                response.success = True
                response.message = "Inference enabled for front camera"
                self.get_logger().info("Inference enabled for front camera")
                
            elif request.camera_mode == SetInferenceCamera.Request.BOTTOM_CAMERA:
                self.inference_enabled = True
                self.camera_mode = request.camera_mode
                response.success = True
                response.message = "Inference enabled for bottom camera"
                self.get_logger().info("Inference enabled for bottom camera")
                
            else:
                response.success = False
                response.message = f"Invalid camera mode: {request.camera_mode}. Valid modes: 0=disabled, 1=front, 2=bottom"
                
        except Exception as e:
            response.success = False
            response.message = f"Service error: {str(e)}"
            
        return response

    def load_model(self):
        """Load the ONNX model"""
        if not os.path.isfile(self.model_path):
            self.get_logger().error(f"ONNX model not found: {self.model_path}")
            raise FileNotFoundError(f"ONNX model not found: {self.model_path}")

        try:
            self.session = ort.InferenceSession(
                self.model_path, providers=self.providers
            )

            # Inspect output names & shapes
            outs = self.session.get_outputs()
            names_shapes = [(o.name, o.shape) for o in outs]

            if self.debug:
                self.get_logger().debug("ONNX outputs:")
                for n, s in names_shapes:
                    self.get_logger().debug(f"  {n} shape={s}")

            # Single-input name
            self.input_name = self.session.get_inputs()[0].name

            # Determine model type based on number of outputs
            if len(outs) == 3:
                # Standard seg export: [det, coefs, proto]
                det3, coef3, proto3 = outs
                self.det_name = det3.name
                self.coef_name = coef3.name
                self.proto_name = proto3.name
                self.mode_2out = False
                self.get_logger().info("Loaded 3-output segmentation model")

            elif len(outs) == 2:
                # Raw export: [raw_preds, proto]
                raw, proto = outs
                self.raw_name = raw.name
                self.proto_name = proto.name
                self.det_name = None
                self.coef_name = None
                self.mode_2out = True
                self.get_logger().info("Loaded 2-output raw model")

            else:
                raise ValueError(f"Expected 2 or 3 outputs, got: {names_shapes}")

        except Exception as e:
            self.get_logger().error(f"Failed to load ONNX model: {e}")
            raise

    def apply_depth_masking(self, img: np.ndarray, depth: np.ndarray) -> np.ndarray:
        """Apply depth-based masking to turn distant pixels blue"""
        img_masked = img.copy()
        
        # Create mask for pixels at 10+ meters (assuming depth is in meters)
        distant_mask = depth >= 10.0
        
        # Set distant pixels to blue (BGR format: [255, 0, 0])
        img_masked[distant_mask] = [255, 90, 90]
        
        return img_masked

    def preprocess(self, img: np.ndarray, depth: Optional[np.ndarray] = None) -> Tuple[np.ndarray, float, int, int]:
        """Preprocess image for ONNX inference"""
        # Apply depth masking if depth data is available
        if depth is not None:
            img = self.apply_depth_masking(img, depth)
        
        h0, w0 = img.shape[:2]
        scale = self.input_size / max(h0, w0)
        nw, nh = int(w0 * scale), int(h0 * scale)
        img_resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)

        # Padding
        pad_w, pad_h = self.input_size - nw, self.input_size - nh
        top, bot = pad_h // 2, pad_h - pad_h // 2
        left, right = pad_w // 2, pad_w - pad_w // 2
        img_padded = cv2.copyMakeBorder(
            img_resized,
            top,
            bot,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )

        # BGR→RGB, HWC→CHW, normalize
        img_preprocessed = (
            img_padded[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
        )
        return np.expand_dims(img_preprocessed, 0), scale, left, top

    def postprocess_onnx(
        self,
        det,
        coefs,
        proto,
        orig_shape: Tuple[int, int],
        scale: float,
        pad_x: int,
        pad_y: int,
    ) -> np.ndarray:
        """Postprocess ONNX inference results to create segmentation mask"""

        # Remove batch dimension
        det = det[0]
        coefs = coefs[0]
        proto = proto[0]  # (M, Hp, Wp)

        if self.debug:
            self.get_logger().debug(f"DEBUG: det shape: {det.shape}")
            self.get_logger().debug(f"DEBUG: coefs shape: {coefs.shape}")
            self.get_logger().debug(f"DEBUG: proto shape: {proto.shape}")
            self.get_logger().debug(
                f"DEBUG: confidence scores range: {det[:, 4].min():.4f} - {det[:, 4].max():.4f}"
            )

        # Filter by confidence
        scores = det[:, 4]
        keep = scores > self.conf_threshold
        det_filtered = det[keep]
        coefs_filtered = coefs[keep]

        if self.debug:
            self.get_logger().debug(
                f"DEBUG: detections after confidence filtering ({self.conf_threshold}): {len(det_filtered)}"
            )

        # Apply top-k filtering if specified
        if self.top_k is not None and len(det_filtered) > self.top_k:
            conf_indices = np.argsort(det_filtered[:, 4])[::-1][: self.top_k]
            det_filtered = det_filtered[conf_indices]
            coefs_filtered = coefs_filtered[conf_indices]

            if self.debug:
                self.get_logger().debug(
                    f"DEBUG: Keeping only top {self.top_k} most confident detections"
                )

        # Initialize combined mask
        H0, W0 = orig_shape[:2]
        combined_mask = np.zeros((H0, W0), dtype=np.float32)

        if len(det_filtered) == 0:
            if self.debug:
                self.get_logger().debug(
                    "DEBUG: No detections found! Returning empty mask."
                )
            return combined_mask

        # Decode masks
        M, Hp, Wp = proto.shape
        proto_flat = proto.reshape(M, -1)
        masks = coefs_filtered @ proto_flat
        masks = masks.reshape(-1, Hp, Wp)

        for i, d in enumerate(det_filtered):
            x1, y1, x2, y2, conf, cls = d

            if self.debug:
                self.get_logger().debug(
                    f"DEBUG: Detection {i+1}: bbox=({x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f}), conf={conf:.4f}, class={int(cls)}"
                )

            # Process mask
            mask_logit = masks[i]
            mask = sigmoid(mask_logit)

            # # Handle aspect ratio correction
            # corrected_height = int(H0*1.333)
            # mask_resized = cv2.resize(mask, (W0, corrected_height))

            # if corrected_height > H0:
            #     start_y = (corrected_height - H0) // 2
            #     end_y = start_y + H0
            #     mask_resized = mask_resized[start_y:end_y, :]
            
            # upscale from 160x160 (model output) to 640x640 (input image)
            mask_net = cv2.resize(mask, (self.input_size, self.input_size))

            #remove the padding added
            x0, y0 = pad_x, pad_y
            scaled_w, scaled_h = int(W0 * scale), int(H0 * scale) #size of the image after scaling in preprocessing
            y1 = pad_y + scaled_h
            x1 = pad_x + scaled_w
            mask_unpadded = mask_net[y0:y1, x0:x1]

            # resize back to original image size
            mask_resized = cv2.resize(mask_unpadded, (W0, H0))  


            # Create binary mask and add to combined mask
            # Using class ID + 1 to differentiate between classes (0 = background)
            bin_mask = (mask_resized > self.mask_threshold).astype(np.float32)
            class_mask = bin_mask * (int(cls) + 1)

            # Combine masks (take maximum to avoid overwriting)
            combined_mask = np.maximum(combined_mask, class_mask)

        return combined_mask

    def inference(self, rgb, depth):
        """
        Main inference method called by ObjectDetectorNode

        Args:
            rgb: RGB image as numpy array (H, W, 3)
            depth: Depth image as numpy array (H, W)

        Returns:
            label_img: Segmentation mask as numpy array (H, W) with class IDs
        """
        try:
            if self.debug:
                self.get_logger().debug(f"DEBUG: Input RGB shape: {rgb.shape}")
                if depth is not None:
                    self.get_logger().debug(f"DEBUG: Input depth shape: {depth.shape}")

            # Preprocess image
            inp, scale, pad_x, pad_y = self.preprocess(rgb, depth)

            if self.debug:
                self.get_logger().debug(f"DEBUG: Preprocessed input shape: {inp.shape}")
                self.get_logger().debug(f"DEBUG: Scale factor: {scale}")
                self.get_logger().debug(f"DEBUG: Padding: x={pad_x}, y={pad_y}")
                self.get_logger().debug(
                    f"Providers being used: {self.session.get_providers()}"
                )

            # Run ONNX inference 
            with self.model_lock:
                outs = self.session.run(None, {self.input_name: inp})

            if self.mode_2out:
                # Handle 2-output model (raw predictions + proto)
                raw_preds, proto = outs
                _, C, N = raw_preds.shape
                M = proto.shape[1]

                if self.debug:
                    self.get_logger().debug(
                        f"DEBUG: Raw predictions shape: {raw_preds.shape}"
                    )
                    self.get_logger().debug(f"DEBUG: Proto shape: {proto.shape}")

                # Transpose raw → (1,N,C)
                raw = raw_preds.transpose(0, 2, 1)

                # Split channels
                xywh = raw[..., :4]
                obj_conf = raw[..., 4:5]

                # Calculate number of classes dynamically
                num_classes = (
                    C - 5 - M
                )  # Total - bbox(4) - objectness(1) - mask_coeffs(M)

                if self.debug:
                    self.get_logger().debug(
                        f"DEBUG: Calculated number of classes: {num_classes}"
                    )

                if num_classes <= 0:
                    # Single class model
                    cls_scores = obj_conf
                    cls_id = np.zeros_like(obj_conf)
                    mask_coefs = raw[..., 5:]
                else:
                    # Multi-class model
                    cls_scores = raw[..., 5 : 5 + num_classes]
                    mask_coefs = raw[..., 5 + num_classes :]
                    cls_id = np.expand_dims(cls_scores.argmax(-1), axis=-1)
                    cls_conf = np.expand_dims(cls_scores.max(-1), axis=-1)
                    obj_conf = obj_conf * cls_conf

                # Decode bounding boxes
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

                # Combine into detection format
                det = np.concatenate([x1, y1, x2, y2, obj_conf, cls_id], axis=-1)
                coefs = mask_coefs

            else:
                # Handle 3-output model (standard segmentation export)
                det, coefs, proto = outs[0], outs[1], outs[2]

            # Create segmentation mask
            label_img = self.postprocess_onnx(
                det, coefs, proto, rgb.shape, scale, pad_x, pad_y
            )

            if self.debug:
                self.get_logger().debug(
                    f"DEBUG: Output label_img shape: {label_img.shape}"
                )
                self.get_logger().debug(
                    f"DEBUG: Unique values in mask: {np.unique(label_img)}"
                )
            """
            if self.debug:
                display_mask = (label_img > 0).astype(np.uint8) * 255
                cv2.imshow("ONNX Segmentation Mask", display_mask)
                cv2.waitKey(1)
            """
            return label_img

        except Exception as e:
            self.get_logger().error(f"Error during inference: {e}")
            # Return empty mask on error
            return np.zeros((rgb.shape[0], rgb.shape[1]), dtype=np.float32)


# FIXME make the mask return in 32SC1 format, instead of 32FC1


def main(args=None):
    rclpy.init(args=args)
    node = OnnxSegmentationDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
