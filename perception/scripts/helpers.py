from typing import Any

def process_result(result: Any) -> dict:
    """
    Processes the YOLO model prediction result and extracts relevant information.

    Args:
        result (Any): An object containing the YOLO model prediction result.

    Returns:
        dict: A dictionary containing the following information:
            - 'detected': A boolean indicating whether any objects were detected.
            - 'max_area': A float representing the area of the largest detected bounding box.
            - 'num_boxes': An integer representing the number of detected bounding boxes.
            - 'confidence': A float representing the confidence of the bounding box with the largest area.
            - 'percentage_cover': A float representing the percentage cover of the box with the largest area.


    Usage:
        >>> result = <object containing the YOLO prediction results>  {
            'boxes': {
                'xywh': ...,
                'conf': ...
            }
            'keys': ...,
            'names': ...,
        }
        >>> info = process_result(result)
        >>> print(info)
        {
            'detected': True,
            'max_area': 5000.0,
            'num_boxes': 2,
            'confidence': 0.95
        }
    """
    boxes = result.boxes

    # Check if detected or not
    detected = boxes.shape[0] > 0

    # Initialize variables for maximum area and confidence
    max_area = 0.0
    max_confidence = 0.0
    percentage_cover = 0.0

    if detected:
        # Extract width and height from the xywh tensor
        widths = boxes.xywh[:, 2]
        heights = boxes.xywh[:, 3]

        # Compute areas and find the maximum area
        areas = widths * heights
        max_area_index = areas.argmax()
        max_area = areas[max_area_index].item()
        max_confidence = boxes.conf[max_area_index].item()

        # Calculate the total area of the original image
        orig_image_area = result.orig_shape[0] * result.orig_shape[1]

        # Calculate the percentage cover of the box with the largest area
        percentage_cover = (max_area / orig_image_area) * 100

    # Number of boxes
    num_boxes = boxes.shape[0]

    # Create object with desired information
    object_info = {
        'detected': detected,
        'max_area': max_area,
        'num_boxes': num_boxes,
        'confidence': max_confidence,
        'percentage_cover': percentage_cover
    }

    return object_info



def process_peg_pose_estimator_result(result: Any) -> dict:
    """
    Processes the YOLO model prediction result and extracts relevant information.

    Args:
        result (Any): An object containing the YOLO model prediction result.

    Returns:
        dict: A dictionary containing the following information:
            - 'detected': A boolean indicating whether any objects were detected.
            - 'max_area': A float representing the area of the largest detected bounding box.
            - 'num_boxes': An integer representing the number of detected bounding boxes.
            - 'confidence': A float representing the confidence of the bounding box with the largest area.
            - 'percentage_cover': A float representing the percentage cover of the box with the largest area.


    Usage:
        >>> result = <object containing the YOLO prediction results>  {
            'boxes': {
                'xywh': ...,
                'conf': ...
            }
            'keys': ...,
            'names': ...,
        }
        >>> info = process_result(result)
        >>> print(info)
        {
            'detected': True,
            'max_area': 5000.0,
            'num_boxes': 2,
            'confidence': 0.95
        }
    """
    boxes = result.boxes
    keypoints = result.keypoints

    # Check if detected or not
    detected = boxes.shape[0] > 0

    # Initialize variables for box and keypoints
    box_xywh = []
    keypoints_xy = [[],[]]

    if detected:
        # Convert tensor to list and extract xywh tensor values
        box_xywh = boxes.xywh[0].tolist()
        # Flatten the nested array for keypoints
        keypoints_xy = [[round(coord_item, 1) for coord_item in coord] for coord in keypoints.xy[0].tolist()]

    # Create object with desired information
    object_info = {
        'detected': detected,
        'box_xywh': box_xywh,
        'keypoints_xy': keypoints_xy,
    }

    return object_info


