create table face_info (
	id serial primary key,
	position_x float8,
	position_y float8,
	position_z float8,
	generic_id int,
	generic_name varchar(255),
	generic_score float8,
	specific_id int,
	specific_name varchar(255),
	specific_score float8,
	width int,
	height int,
	age int,
	gender varchar(255),
	bgr bytea,
	ros_timestamp float8
	);
