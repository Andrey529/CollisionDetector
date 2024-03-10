
def format_output_data(numbers):
	left_inf, right_inf, left_lidar, right_lidar = numbers
	left_inf = int(left_inf)
	right_inf = int(right_inf)
	left_lidar = int(left_lidar)
	right_lidar  = int(right_lidar)
	first_sector = inf_calculation(left_inf)
	second_sector = lidar_calculation(left_lidar)
	third_sector = lidar_calculation(right_lidar)
	fourth_sector = inf_calculation(right_inf)
	#0 - white
	#1 - green
	#2 - red
	output_data = [1] * 8
	for i in range(8):
		output_data[i] = [1] * 8
	rows = [0,1,2]
	columns = [0,1,6,7]
	for i in rows:
		for j in columns:
			output_data[i][j] = 0
	
	for i in range(3,3+first_sector):
		output_data[i][0] = 2
		output_data[i][1] = 2
	for i in range(3,3+fourth_sector):
		output_data[i][6] = 2
		output_data[i][7] = 2
	for i in range(second_sector):
		output_data[i][2] = 2
		output_data[i][3] = 2
	for i in range(third_sector):
		output_data[i][4] = 2
		output_data[i][5] = 2
	return output_data
	
	
	
	
def lidar_calculation(distance):
	if distance == 0:
		return 0
	points_in_lidar_zone = 3
	# calculate by inf radar zone if less then 450sm
	if distance < 450:
		return points_in_lidar_zone + inf_calculation(distance)
	
	#TODO check can lidar give more then 1200sm
	
	# calculate by lidar zone if more then 450sm
	max_distance_in_inf_zone = 450
	proportion_by_one_point = 250
	distance_in_lidar_zone = distance - max_distance_in_inf_zone
	return points_in_lidar_zone - distance_in_lidar_zone // proportion_by_one_point
	
def inf_calculation(distance):
	if distance == 0:
		return 0
	points_in_sector = 5
	proportion_by_one_point = 90
	max_distance_in_inf_zone = 450
	# too high distance
	if distance > max_distance_in_inf_zone:
		return 0
	# calculate how many points will be drown
	return points_in_sector - distance // proportion_by_one_point

def check_res():
	data = [0]*4
	data[0] = [700,700,900,300]
	data[1] = [0,0,0,0]
	data[2] = [300,50,500,1000]
	data[3] = [50,300,900,600]
	for example in data:
		print()
		print(example)
		format_output_data(example)
		print()
