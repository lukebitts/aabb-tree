/*
This is an example demonstrating how to use an AABB tree to detect collisions. The example is
visual, it uses piston to show a screen with random blue circles moving around, whenever these
circles collide, they change color.
*/

extern crate aabb_tree;
extern crate glm;
extern crate rand;
extern crate chrono;

extern crate piston_window;

use piston_window::*;
use chrono::*;

use glm::Vector3;
use aabb_tree::AabbTree;

// We keep the state of our circles here
#[derive(Debug, Clone)]
struct Circle {
	radius: f32,
	position: Vector3<f32>,
	velocity: Vector3<f32>,
	colliding: bool
}

fn main() {

	// Create a new `AabbTree` of `Circle`s
	let mut tree : AabbTree<Circle> = AabbTree::new();

	// Whenever we add something to the tree, we get a proxy back. Let's keep them in this list:
	let mut proxies = Vec::new();

	// This `for` is an overengineered way to create a random `Circle`, add it to the tree and add
	// the proxy to the proxies list.
	for _ in 0..20 {
		let initial_position = Vector3::new(rand::random::<f32>() * 1024.0, rand::random::<f32>() * 800.0, 0.0);
		let radius = if rand::random() { 20.0 + rand::random::<f32>() * 40.0 } else { 20.0 };
		let min = initial_position - Vector3::new(radius, radius, radius);
		let max = initial_position + Vector3::new(radius, radius, radius);

		let mut vx = if rand::random() { 300.0 } else { -300.0 };
		let mut vy = if rand::random() { 300.0 } else { -300.0 };
		if rand::random() { vx += vx * 0.5; }
		if rand::random() { vy += vy * 0.5; }

		// To add something to the tree, you need an AABB, which is defined as `((f32, f32, f32), (f32, f32, f32))`.
		proxies.push(tree.create_proxy(
			((min.x, min.y, min.z), (max.x, max.y, max.z)),
			Circle { radius: radius, position: initial_position, velocity: Vector3::new(vx, vy, 0.0), colliding: false }
		));
	}

	// Window creation
	let mut window: PistonWindow =
	WindowSettings::new("Collision example", [1024, 800])
	.exit_on_esc(true).build().unwrap();

	let mut last_frame = UTC::now();

	while let Some(e) = window.next() {

		// Delta time calculations
		let now = UTC::now();
		let delta = (now - last_frame).num_milliseconds() as f64 / 1000.0;
		last_frame = now;

		// This `for` is used to move the circles around, basically they move until they hit one of
		// the sides of the screen and then reverse their direction.
		for proxy in &proxies {
			let (pos, radius) = {
				let circle = tree.user_data_mut(*proxy).unwrap();

				circle.position.x += (circle.velocity.x as f64 * delta) as f32;
				circle.position.y += (circle.velocity.y as f64 * delta) as f32;

				if circle.position.x > 1024.0 {
					circle.position.x = 1024.0; circle.velocity.x *= -1.0;
				}
				else if circle.position.x < 0.0 {
					circle.position.x = 0.0; circle.velocity.x *= -1.0;
				}
				if circle.position.y > 800.0 {
					circle.position.y = 800.0; circle.velocity.y *= -1.0;
				}
				else if circle.position.y < 0.0 {
					circle.position.y = 0.0; circle.velocity.y *= -1.0;
				}

				(circle.position, circle.radius)
			};

			// Once we move the circle we need to update its AABB inside the tree.
			let min = pos - Vector3::new(radius, radius, radius);
			let max = pos + Vector3::new(radius, radius, radius);
			let aabb = ((min.x, min.y, min.z), (max.x, max.y, max.z));

			// `AabbTree::set_aabb` can trigger a tree rebalance if the new AABB is not contained
			// by the proxy's parent AABB
			tree.set_aabb(*proxy, &aabb);

			// We query the tree to find collisions
			let mut colliding = false;
			tree.query_aabb(&aabb, |colliding_proxy| {
				// Since we are querying an area inside the tree, we have to filter the results
				// to exclude the current proxy we are testing. We return true because we wan't
				// to keep searching for other collisions.
				if colliding_proxy == *proxy { return true; }

				let coll = tree.user_data(colliding_proxy).unwrap();
				let this = tree.user_data(*proxy).unwrap();

				// Simple circle collision
				colliding = glm::distance(coll.position, this.position) < (coll.radius + this.radius);

				// Since we don't care about extra collisions, once we found our first one we can
				// stop looking.
				!colliding
			});

			// Set the collision result, so we can render this in different colors.
			tree.user_data_mut(*proxy).unwrap().colliding = colliding;
		}

		window.draw_2d(&e, |c, g| {
			use rectangle::centered_square as circle;

			clear([1.0; 4], g);

			// Render every circle
			for proxy in &proxies {
				let ci = tree.user_data(*proxy).unwrap();

				let color = if ci.colliding {
					[0.0, 0.5, 1.0, 1.0]
				}
				else {
					[0.0, 0.0, 1.0, 1.0]
				};

				ellipse(color,
					circle(ci.position.x as f64, ci.position.y as f64, ci.radius as f64),
					c.transform, g);
			}
		});

	}

}
