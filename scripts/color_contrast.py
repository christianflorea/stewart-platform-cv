import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# =============
# TEST RESULTS:
# =============


ping_ping_white = [
    (146, 96, 32),
    (163, 125, 53),
    (166, 126, 54),
    (149, 100, 32),
    (160, 124, 46),
    (153, 113, 45),
    (218, 162, 74),
    (249, 204, 157),
    (214, 156, 75),
    (239, 186, 112),
    (214, 159, 77),
    (238, 181, 105),
]

ping_ping_black = [
    (213, 181, 136),
    (203, 166, 102),
    (188, 152, 94),
    (201, 159, 99),
    (190, 158, 118),
    (176, 137, 87),
    (209, 177, 128),
    (202, 164, 110),
    (145, 110, 62),
    (132, 96, 44),
    (142, 108, 58),
    (125, 78, 32),
]

ball_bearing_white = [
    (127, 129, 128),
    (91, 93, 125),
    (152, 155, 164),
    (31, 28, 30),
    (103, 106, 104),
    (2, 0, 2),
    (69, 67, 56),
    (45, 50, 58),
    (113, 112, 113),
    (74, 67, 62),
    (13, 11, 12),
    (70, 68, 57),
]

ball_bearing_black = [
    (149, 142, 133),
    (145, 141, 137),
    (131, 129, 135),
    (114, 108, 101),
    (136, 130, 121),
    (148, 145, 146),
    (148, 145, 146),
    (174, 170, 181),
    (149, 133, 128),
    (169, 169, 196),
    (123, 109, 108),
    (68, 55, 53),
]


# ========================
# AVERAGE Colour OF BALLS:
# ========================


def get_colour_avg(rgb_list):
    r_sum, g_sum, b_sum = map(sum, zip(*rgb_list))
    colour_len = len(rgb_list)
    return (r_sum // colour_len, g_sum // colour_len, b_sum // colour_len)

# PING PONG BALL

ping_ping_white_avg = get_colour_avg(ping_ping_white)
ping_ping_black_avg = get_colour_avg(ping_ping_black)
ping_ping_avg = get_colour_avg([ping_ping_white_avg, ping_ping_black_avg])

print("Ping Pong - White Background Average:", ping_ping_white_avg)
print("Ping Pong - Black Background Average:", ping_ping_black_avg)
print("Ping Pong Colour: ", ping_ping_avg)

# BALL BEARING BALL

ball_bearing_white_avg = get_colour_avg(ball_bearing_white)
ball_bearing_black_avg = get_colour_avg(ball_bearing_black)
ball_bearing_avg = get_colour_avg([ball_bearing_white_avg, ball_bearing_black_avg])

print("Ball Bearing - White Background Average:", ball_bearing_white_avg)
print("Ball Bearing - Black Background Average:", ball_bearing_black_avg)
print("Ball Bearing Colour: ", ball_bearing_avg)


# ====================
# MAXIMIZING CONTRAST:
# ====================

golf_ball_avg = (237, 241, 230)
ball_colours = [ping_ping_avg, ball_bearing_avg]

def average_contrast(origin, colour_list):
    """
    Calculates the average Euclidean distance (contrast) between origin colour and each colour in list.
    """
    distances = []
    for c in colour_list:
        dist = ((origin[0] - c[0]) ** 2 + (origin[1] - c[1]) ** 2 + (origin[2] - c[2]) ** 2) ** 0.5
        distances.append(dist)
    return round(sum(distances) / len(distances), 2)

max_contrast = -1
best_colour = None

# List of colours that are being considered
possible_colours = []

# Generate all possible colours
accuracy = 4
for r in range(0, 256, accuracy):
    for g in range(0, 256, accuracy):
        for b in range(0, 256, accuracy):
            possible_colours.append((r, g, b))

accuracy_to_colour_list = []

for c in possible_colours:
  avg_contrast = average_contrast(c, ball_colours)
  accuracy_to_colour_list.append((avg_contrast, c))
  if avg_contrast > max_contrast:
      max_contrast = avg_contrast
      best_colour = c

print("Colour with maximum average contrast:", best_colour)
print("Maximum average contrast:", max_contrast)


# ====================
# Visualizing Results:
# ====================

accuracy_to_colour_list.sort(reverse=True)
# Get the top n colours
n = 10
accuracy_to_colour_list = accuracy_to_colour_list[:n]

num_columns = 5
num_rows = (n + num_columns - 1) // num_columns

# Create the plot
fig, ax = plt.subplots(figsize=(num_columns, num_rows))

ax.set_xlim(0, num_columns)
ax.set_ylim(0, num_rows)

# Remove axes ticks and spines
ax.set_xticks([])
ax.set_yticks([])
for spine in ax.spines.values():
    spine.set_visible(False)

# Display the colors and contrast values
for idx, (contrast, rgb) in enumerate(accuracy_to_colour_list):
    row = idx // num_columns
    col = idx % num_columns
    r, g, b = rgb
    color = (r / 255.0, g / 255.0, b / 255.0)

    rect = Rectangle((col, num_rows - row - 1), 1, 1, color=color)
    ax.add_patch(rect)

    contrast_text = f"{contrast:.1f}"
    rgb_text = f"({r}, {g}, {b})"
    ax.text(col + 0.5, num_rows - row - 0.6, contrast_text, ha='center', va='center', fontsize=6, color='black')
    ax.text(col + 0.5, num_rows - row - 0.4, rgb_text, ha='center', va='center', fontsize=6, color='black')

plt.tight_layout()
plt.show()