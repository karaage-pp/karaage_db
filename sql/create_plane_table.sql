create table plane_info (
	id serial primary key,

	center_x float8,
	center_y float8,
	center_z float8,

	normal_x float8,
	normal_y float8,
	normal_z float8,

	upperLeft_x float8,
	upperLeft_y float8,
	upperLeft_z float8,

	lowerLeft_x float8,
	lowerLeft_y float8,
	lowerLeft_z float8,

	upperRight_x float8,
	upperRight_y float8,
	upperRight_z float8,

	lowerRight_x float8,
	lowerRight_y float8,
	lowerRight_z float8,

	ros_timestamp float8
	);
