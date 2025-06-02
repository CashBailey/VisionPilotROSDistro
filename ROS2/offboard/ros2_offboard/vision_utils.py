import numpy as np
import cv2
from numpy.linalg import inv, svd

def unitize(x, y):
    magnitude = np.sqrt(x**2 + y**2)
    l = x / magnitude
    m = y / magnitude
    return l, m

def vex(A):
    if A.shape != (3, 3):
        raise ValueError("vex: expects 3x3 matrices as an input")
    v = np.zeros(3)
    v[0] = 0.5 * (A[2, 1] - A[1, 2])
    v[1] = 0.5 * (A[0, 2] - A[2, 0])
    v[2] = 0.5 * (A[1, 0] - A[0, 1])
    return v

def compute_homography(cv_reference_points, cv_current_points, K):
    cv_homography, _ = cv2.findHomography(cv_reference_points, cv_current_points, cv2.RANSAC)
    homography = np.dot(np.dot(inv(K), cv_homography), K)
    return homography

def recover_from_homography(homography, K, current_iteration, homography_solution):
    U, S, Vt = svd(homography)
    V = Vt.T

    s1 = S[0] / S[1]
    s3 = S[2] / S[1]
    zeta = s1 - s3
    if abs(zeta) < np.finfo(float).eps:
        distance_plane = 1.0
        R = np.eye(3)
        n = np.zeros(3)
        n[0] = 1.0
        t = np.zeros(3)
        t[0] = 1.0
        return R, t, n, distance_plane, homography_solution

    a1 = np.sqrt(1 - s3**2)
    b1 = np.sqrt(s1**2 - 1)
    a, b = unitize(a1, b1)
    c, d = unitize(1 + s1 * s3, a1 * b1)
    e, f = unitize(-b / s1, -a / s3)

    v1 = V[:, 0]
    v3 = V[:, 2]

    n1 = b * v1 - a * v3
    n2 = b * v1 + a * v3

    tmp = np.array([[c, 0.0, d],
                    [0, 1.0, 0],
                    [-d, 0.0, c]])

    R1 = np.dot(U, np.dot(tmp, V.T))
    R2 = np.dot(U, np.dot(tmp.T, V.T))

    t1 = e * v1 + f * v3
    t2 = e * v1 - f * v3

    if n1[2] < 0:
        t1 = -t1
        n1 = -n1

    if n2[2] < 0:
        t2 = -t2
        n2 = -n2

    if current_iteration == 0:
        if n1[2] > n2[2]:
            R = R1
            t = t1
            n = n1
            homography_solution = "SOLUTION_1"
        else:
            R = R2
            t = t2
            n = n2
            homography_solution = "SOLUTION_2"
    else:
        if homography_solution == "SOLUTION_1":
            R = R1
            t = t1
            n = n1
        else:
            R = R2
            t = t2
            n = n2

    distance_plane = 1.0 / zeta
    return R, t, n, distance_plane, homography_solution

def estimate_homography(ref_img, curr_img, K, counter):
    n_features = 600
    orb = cv2.ORB_create(n_features)

    keypoints_ref, descriptors_ref = orb.detectAndCompute(ref_img, None)
    keypoints_curr, descriptors_curr = orb.detectAndCompute(curr_img, None)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(descriptors_ref, descriptors_curr)
    matches = sorted(matches, key=lambda x: x.distance)

    good_matches = matches[:50]

    ref_points = np.float32([keypoints_ref[m.queryIdx].pt for m in good_matches])
    curr_points = np.float32([keypoints_curr[m.trainIdx].pt for m in good_matches])

    if len(ref_points) < 4 or len(curr_points) < 4:
        print("Not enough points to estimate the homography")
        return None

    H, _ = cv2.findHomography(ref_points, curr_points, cv2.RANSAC, 5.0)
    H = np.dot(np.dot(inv(K), H), K)
    H /= H[1, 1]

    return H

def rodriguez(R):
    u, _ = cv2.Rodrigues(R)
    return u

def pbvs_controller(R, t, u, lambdav, lambdaw):
    Uv = -lambdav * np.dot(R.T, t)
    Uw = -lambdaw * u
    return Uv, Uw

def normalize_point(point, Kinv):
    norm_point = np.dot(Kinv, np.hstack((point, [1])))
    return norm_point[:2] / norm_point[2]
