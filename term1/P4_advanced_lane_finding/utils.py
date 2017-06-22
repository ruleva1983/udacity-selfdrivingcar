import cv2
import numpy as np

def superimpose_images(original_image, lane, Minv, curvature):
    img_lines = np.zeros_like(original_image)
    img_free_space = np.zeros_like(original_image)

    # Draw lines
    img_lines, _ = lane.draw_lines(img_lines, thick=50, color=(255, 0, 0))

    # Draw free space
    img_free_space, _ = lane.draw_free_space(img_free_space, (0, 255, 0))

    # Unwarp images
    img_free_space_unwarp = cv2.warpPerspective(img_free_space, Minv,
                                                (original_image.shape[1], original_image.shape[0]))
    img_lines_unwarp = cv2.warpPerspective(img_lines, Minv, (original_image.shape[1], original_image.shape[0]))

    # Blend with original image
    img_out = cv2.addWeighted(original_image, 1, img_lines_unwarp, 0.3, 0)
    img_out = cv2.addWeighted(img_out, 1, img_free_space_unwarp, 0.3, 0)

    font = cv2.FONT_HERSHEY_SIMPLEX
    color = (255, 255, 255)
    scale = 2
    thickness = 2
    cv2.putText(img_out, "Left line curvature: %.1f m" % curvature[0], (100, 50), font, scale, color, thickness)
    cv2.putText(img_out, "Right line curvature: %.1f m" % curvature[1], (100, 100), font, scale, color, thickness)
    distance = lane.relative_car_position()
    if distance < 0:
        cv2.putText(img_out, "Position: %.1f m right of the center" % distance, (100, 150), font, scale, color,
                    thickness)
    else:
        cv2.putText(img_out, "Position: %.1f m left of the center" % distance, (100, 150), font, scale, color,
                    thickness)
    return img_out