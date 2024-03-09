import output_matrix_calculation
import matrix_visualisation
def check_res():
	data = [0]*4
	data[0] = [700,700,900,300]
	data[1] = [0,0,0,0]
	data[2] = [300,50,500,1000]
	data[3] = [50,300,900,600]
	for example in data:
		print()
		print(example)
		matrix_visualisation.print_data(output_matrix_calculation.format_output_data(example))
		print("break power: ",calculate_break_power(example))
		print()
		
def calculate_break_power(numbers):
	left_inf, right_inf, left_lidar, right_lidar = numbers
	left_inf = int(left_inf)
	right_inf = int(right_inf)
	left_lidar = int(left_lidar)
	right_lidar  = int(right_lidar)
	first_sector = output_matrix_calculation.inf_calculation(left_inf)
	second_sector = output_matrix_calculation.lidar_calculation(left_lidar)
	third_sector = output_matrix_calculation.lidar_calculation(right_lidar)
	fourth_sector = output_matrix_calculation.inf_calculation(right_inf)
	inf_sector_break_proportion = 20
	lidar_sector_break_proportion = 12.5
	break_lvl_inf_sectors = max(first_sector,fourth_sector) * inf_sector_break_proportion
	break_lvl_lidar_sectors = int(max(second_sector,third_sector) * lidar_sector_break_proportion)
	
	return max(break_lvl_inf_sectors,break_lvl_lidar_sectors)

check_res()
