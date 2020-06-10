function Body(id, x_pos, y_pos, x_vel, y_vel, r) {
	this.id = id;
	this.x_pos = x_pos;
	this.y_pos = y_pos;
	this.x_vel = x_vel;
	this.y_vel = y_vel;
	this.radius = r;
	this.mass = 0.1 * Math.pow(r,3); 
};

Body.prototype.update = function(dt, bodies){
	//position and velocity updating employs leapfrog integration

	this.updateForce(bodies);
	
	this.x_vel = this.x_vel + this.x_acc*dt/2;
	this.y_vel = this.y_vel + this.y_acc*dt/2;
	
	this.x_pos = this.x_pos + this.x_vel*dt;
	this.y_pos = this.y_pos + this.y_vel*dt;
	
	this.updateForce(bodies);
	
	this.x_vel = this.x_vel + this.x_acc*dt/2;
	this.y_vel = this.y_vel + this.y_acc*dt/2;
};

Body.prototype.draw = function(){
	let svg = document.getElementById("svg");
	let circle = svg.getElementById(this.id);
	circle.setAttributeNS(null, 'cx', this.x_pos);
	circle.setAttributeNS(null, 'cy', this.y_pos);
	circle.setAttributeNS(null, 'r', this.radius);
}

Body.prototype.detectCollision = function(bodies){
	let x_dist;
	let y_dist;	
	let combined_radii;
	let seperation_magnitude;
	let p_x;
	let p_y;
	for(i=0; i<bodies.length; i++){
		x_dist = this.x_pos - bodies[i].x_pos;
		y_dist = this.y_pos - bodies[i].y_pos;
		seperation_magnitude = Math.pow(Math.pow(x_dist,2)+Math.pow(y_dist,2),1/2);	
		combined_radii = parseInt(this.radius) + parseInt(bodies[i].radius);
		if(this.id != bodies[i].id && seperation_magnitude <= combined_radii){
			//transfer momentum
			p_x = bodies[i].mass*bodies[i].x_vel + this.mass*this.x_vel;
			p_y = bodies[i].mass*bodies[i].y_vel + this.mass*this.y_vel;
			bodies[i].x_vel = p_x/(bodies[i].mass + this.mass);		
			bodies[i].y_vel = p_y/(bodies[i].mass + this.mass);		
			//combine mass in one object and change size accordingly
			bodies[i].mass += this.mass;
			bodies[i].radius = Math.pow((10*bodies[i].mass),(1/3));
			console.log("Collision");
			return true;
		}
	}
	return false; 
}

Body.prototype.updateForce = function(bodies){
	let x_dist;
	let y_dist;	
	this.x_acc = 0;	
	this.y_acc = 0;	
	
	for(i=0; i<bodies.length; i++){
		x_dist = this.x_pos - bodies[i].x_pos;
		y_dist = this.y_pos - bodies[i].y_pos;
		if(x_dist != 0 || y_dist != 0){
			this.x_acc +=  -bodies[i].mass * x_dist / Math.pow(Math.pow(x_dist,2)+Math.pow(y_dist,2), 3/2);
			this.y_acc +=  -bodies[i].mass * y_dist / Math.pow(Math.pow(x_dist,2)+Math.pow(y_dist,2), 3/2);
		}
	}
};

