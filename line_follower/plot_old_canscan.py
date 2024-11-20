import numpy as np
import matplotlib.pyplot as plt
plt.show()
# ----------------------------------- split ---------------------------------- #
log_data = np.genfromtxt('angle_distance1.csv', delimiter=',')
distances = log_data[1:, 1]
angles = log_data[1:, 0]

min_distance = min(distances)
print(min_distance)
# indices of min value
min_indices = [index for index, value in enumerate(
    distances) if value == min_distance]
# find middle index of min indices
index = min_indices[len(min_indices)//2]


angles = log_data[1:, 0]
distances = log_data[1:, 1]
plt.plot(angles, distances)
plt.plot(angles[index], min_distance, 'ro')
plt.show()

distances = [(1, 345), (2, 345), (3, 345), (4, 345), (5, 340), (6, 345), (7, 345), (8, 340), (9, 345), (10, 340), (11, 345), (12, 345), (13, 345), (14, 345), (15, 345), (16, 345), (17, 345), (18, 340), (19, 340), (20, 340), (21, 340), (22, 340), (23, 340), (24, 340), (25, 336), (26, 340), (27, 340), (28, 345), (29, 340), (30, 345), (31, 345), (32, 349), (33, 353), (34, 353), (35, 364), (36, 364), (37, 365), (38, 376), (39, 372), (40, 198), (41, 198), (42, 197), (43, 197), (44, 197), (45, 197), (46, 197), (47, 197), (48, 190), (49, 190), (50, 190), (51, 190), (52, 190), (53, 190), (54, 190), (55, 190), (56, 190), (57, 190), (58, 190), (59, 190), (60, 190), (61, 190), (62, 190), (63, 190), (64, 190), (65, 190), (66, 190), (67, 190), (68, 190), (69, 2550), (70, 2550), (71, 2550), (72, 2550), (73, 2550), (74, 2550), (75, 2550), (76, 2550), (77, 2550), (78, 2550), (79, 2550), (80, 2550), (81, 2550), (82, 2550), (83, 2550), (84, 2550), (85, 2550), (86, 2550), (87, 2550), (88, 2550), (89, 2550), (90, 2550), (91, 2550), (92, 2550), (93, 2550), (94, 2550), (95, 2550), (96, 2550), (97, 2550), (98, 2550), (99, 2550), (100, 2550), (101, 2550), (102, 2550), (103, 2550), (104, 2550), (105, 2550), (106, 2550), (107, 2550), (108, 2550), (109, 2550), (110, 2550), (111, 2550), (112, 2550), (113, 2550), (114, 2550), (115, 2550),
             (116, 2550), (117, 2550), (118, 2550), (119, 2550), (120, 2550), (121, 2550), (122, 2550), (123, 2550), (124, 2550), (125, 2550), (126, 2550), (127, 2550), (128, 2550), (129, 2550), (130, 2550), (131, 2550), (132, 2550), (133, 2550), (134, 2550), (135, 2550), (136, 2550), (137, 2550), (138, 1362), (139, 1362), (140, 1362), (141, 1362), (142, 2060), (143, 2060), (144, 2550), (145, 2550), (146, 2550), (147, 2550), (148, 2550), (149, 2550), (150, 1282), (151, 1282), (152, 1282), (153, 1282), (154, 2099), (155, 2099), (156, 2099), (157, 2099), (158, 2099), (159, 2132), (160, 2132), (161, 2132), (162, 2132), (163, 2132), (164, 2147), (165, 2147), (166, 2550), (167, 1854), (168, 2550), (169, 1857), (170, 1857), (171, 2550), (172, 1875), (173, 1875), (174, 2550), (175, 1885), (176, 2550), (177, 2550), (178, 1892), (179, 2550), (180, 1902), (181, 1902), (182, 2550), (183, 1917), (184, 1917), (185, 383), (186, 385), (187, 391), (188, 392), (189, 412), (190, 393), (191, 397), (192, 420), (193, 420), (194, 412), (195, 412), (196, 427), (197, 427), (198, 427), (199, 1978), (200, 452), (201, 452), (202, 452), (203, 452), (204, 1990), (205, 1990), (206, 326), (207, 326), (208, 326), (209, 326), (210, 324), (211, 324), (212, 324), (213, 1856), (214, 2550), (215, 2550), (216, 2550), (217, 2043), (218, 2043), (219, 2550), (220, 2550)]
distances = [distance for angle, distance in distances]
min_distance = min(distances)
# indices of min value
min_indices = [index for index, value in enumerate(
    distances) if value == min_distance]
# find middle index of min indices
index = min_indices[len(min_indices)//2]
print(index)
# Plot the data
plt.plot(distances)
plt.plot(index, min_distance, 'ro')