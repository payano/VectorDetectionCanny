# VectorDetectionCanny

#This is a program for edge detection with OpenCV. It uses Canny's edge detection, after that it creates a graph with one y-coordinate for every x-coordinate. After that the populated list is created it takes out the 30 highest y-coordinate values and copies it to another list. The new list is sorted by the x-value. If three x-coordinates are next to each other(example; [x,y] = [3,10],[4,13],[5,9]) algoritm creates a arrow on the picture with the highes X-value.
