import cv2 
import numpy as np 
 
# Settings untuk chessboard 
pattern_size = (9, 6)  
square_size_mm = 30    
 
# Generate pattern 
pattern = np.ones((297*4, 210*4), dtype=np.uint8) * 255 
for i in range(7): 
    for j in range(10): 
        if (i + j) % 2 == 0: 
            pattern[i*120:(i+1)*120, j*120:(j+1)*120] = 0 
cv2.imwrite("chessboard_9x6_30mm.png", pattern) 
print("Chessboard pattern generated!") 
