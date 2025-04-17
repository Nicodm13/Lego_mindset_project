import cv2
import networkx as nx
import matplotlib.pyplot as plt


def canny_edge_detection(frame):
    # Convert the frame to grayscale for edge detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise and smoothen edges
    blurred = cv2.GaussianBlur(src=gray, ksize=(3, 5), sigmaX=0.5)

    # Perform Canny edge detection
    edges = cv2.Canny(blurred, 70, 135)

    return blurred, edges


if __name__ == "__main__":
    # Open the default webcam
    cap = cv2.VideoCapture(0)
    fps = 1 # Set the desired FPS for the webcam. Go lower if your computer is slow.
    sleepTime = int(1000 / fps)

    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()
        if not ret:
            print('Image not captured')
            break

        # Perform Canny edge detection on the frame
        blurred, edges = canny_edge_detection(frame)

        # Display the original frame and the edge-detected frame
        # cv2.imshow("Original", frame)
        #cv2.imshow("Blurred", blurred)
        cv2.imshow("Edges", edges)
        G = nx.Graph()
        G.add_edges_from(edges)
        plt.show()

        # Exit the loop when 'q' key is pressed
        if cv2.waitKey(sleepTime) & 0xFF == ord('q'):
            break




    # Release the webcam and close the windows
    cap.release()
    cv2.destroyAllWindows()
