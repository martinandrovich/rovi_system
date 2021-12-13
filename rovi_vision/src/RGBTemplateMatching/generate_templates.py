import rospy
import rospkg
import numpy as np
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import SetLinkState
from sensor_msgs.msg import Image
from scipy.spatial.transform import Rotation as R
import os
import json
from tqdm import tqdm
import cv2

z_axis = np.array([0, 0, 1])
image_buffer = None

with open(os.path.dirname(__file__) + "settings.json", "r") as read_file:
	settings = json.load(read_file)
	object_name = settings["object_name"]
	x_lim = settings["x_limits"]
	y_lim = settings["y_limits"]
	z_lim = settings["z_limits"]
	yaw_lim = settings["yaw_limits"]
	cam_topic = settings["camera_topic"]
	template_path = rospkg.RosPack().get_path("rovi_vision") + "/data/templates/" + object_name

def set_obj_pose(name: str, pos: np.ndarray, ori: np.ndarray):
	# maybe change to SetModelState
	link_state = rospy.ServiceProxy("/gazebo/set_link_state", SetLinkState)
	state = LinkState()
	state.pose.position.x = pos[0]; state.pose.position.y = pos[1]; state.pose.position.z = pos[2]
	state.pose.orientation.x = ori[0]; state.pose.orientation.y = ori[1]; state.pose.orientation.z = ori[2]; state.pose.orientation.w = ori[3]
	state.link_name = name
	response = link_state(state)
	return response

def img_callback(img: Image):
	global image_buffer
	image_buffer = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)

if __name__ == "__main__":

	# generate images to train nn
	rospy.init_node("generate_templates")

	# subscribe to link_states and image
	sub_vis = rospy.Subscriber(cam_topic, Image, img_callback, queue_size=1, tcp_nodelay=True)

	# wait until image_buffer is not None
	while image_buffer is None:
		rospy.sleep(0.01)
		
	# object id/link (HARDCODED, YIKES)
	object_id = object_name + "1::link_0"

	# set out of domain object pose
	response = set_obj_pose(object_id, np.array([-500, -500, -500]), np.array([1, 0, 0, 0]))

	# remove path
	# [ os.remove(template_path + file) for file in os.listdir(template_path) if file.endswith(".png") or file.endswith(".xml") ]

	pose_list = []
	x = 0.05
	while x <= 0.75:
		y = 1.0
		while y <= 1.1:
			yaw = 0
			while yaw <= np.pi:
				pose_list.append([x, y, yaw])
				yaw += np.pi/16
			y += 0.025
		x += 0.025

	# get images
	for i in tqdm(range(len(pose_list))):

		# wait some time
		rospy.sleep(0.05)

		# copy image
		if i == 0:
			bg = image_buffer.copy()
		else:
			img = image_buffer.copy()

		if i > 0:

			# kernel for subtraction
			kernel = np.ones((3, 3), np.uint8)
			mask = abs(img - bg) > 2
			mask[:100, :] = 0
			copy = img.copy()

			# mask
			copy[mask == False] = 0
			copy[(mask[:,:,0] == True) | (mask[:, :, 1] == True) | (mask[:, :, 2] == True)] = 255

			# erode and dilate noise
			copy = cv2.erode(copy, kernel, iterations=1)
			copy = cv2.dilate(copy, kernel, iterations=1)

			# bitwise and
			masked_img = np.bitwise_and(img.astype("uint8"), copy.astype("uint8"))
			mask = (masked_img[:, :, 0] == 0)
			masked_img[mask, 0] = 255

			# write to file
			cv2.imwrite(template_path + "/template" + str(i-1).zfill(4) + ".png", cv2.cvtColor(masked_img, cv2.COLOR_BGR2RGB))
			cv_file = cv2.FileStorage(template_path + "/template" + str(i-1).zfill(4) + "_pose.xml", cv2.FILE_STORAGE_WRITE)
			cv_file.write("T", pose.astype("float64"))
			cv_file.release()

		# pose
		pose = np.identity(4)
		pose[:3, :3] = R.from_rotvec(pose_list[i][2] * z_axis).as_matrix()
		pose[:3, 3] = np.array([pose_list[i][0], pose_list[i][1], np.random.uniform(z_lim[0], z_lim[1])])

		# set the object pose
		response = set_obj_pose(object_id, pose[:3, 3], R.from_matrix(pose[:3, :3]).as_quat())

		# if no response exit
		if not response:
			exit()