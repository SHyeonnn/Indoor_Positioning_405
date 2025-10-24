# ToA_estimator.py
import numpy as np

class ToAEstimator:
    def __init__(self):
        self.tag_num = 1
        # width, length in our space
        self.width, self.length = 1.6, 1.6
        self.anchor_num = 4
        # ToAEstimator.anchor_positions 기본 행 순서:
        #   0:[0, L], 1:[0, 0], 2:[W, 0], 3:[W, L] -> 앵커 좌표 처맞추라는 뜻
        # 예) U2→[0,L], U3→[0,0], U4→[W,0], U5→[W,L] 라면:
        self.anchor_positions = np.array([
            [0, self.length],
            [0, 0],
            [self.width, 0],
            [self.width, self.length]
        ])
        self.noise_variance = 1

    def toa_LLS(self, distances):
        if len(distances) != self.anchor_num:
            raise ValueError("Distances Length Error")
        
        ToA_r = np.array(distances).reshape(-1, 1)
        print("Measurement vector = ", ToA_r)

        A = np.hstack((-2*self.anchor_positions, np.ones((self.anchor_num, 1))))
        b = ToA_r ** 2 - np.sum(self.anchor_positions ** 2, axis=1, keepdims=True)

        print("A matrix = ", A)
        print("b vector = ", b)

        thetha_hat = np.linalg.inv(A.T @ A) @ A.T @ b
        x_hat = thetha_hat[0:2].flatten()

        return x_hat
