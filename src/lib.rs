#![deny(missing_docs)]

#![cfg_attr(test, deny(warnings))]
#![cfg_attr(feature="test_tools", feature(plugin))]
#![cfg_attr(feature="test_tools", plugin(clippy))]
#![cfg_attr(feature="test_tools", plugin(quickcheck_macros))]
#![cfg_attr(feature="test_tools", allow(doc_markdown))]

#![cfg_attr(feature="test_tools", feature(plugin, custom_attribute))]

//! # AABB tree
//!
//! This crate is the implementation of a dynamic bounding volume tree based on
//! axis aligned bounding boxes. It is a spatial structure with support for
//! querying objects based on their position and size inside the tree.
//! This work is based on Presson's btDbvt written for Bullet Physics and
//! Catto's `b2DynamicTree` written for Box2D.

#[cfg(feature="test_tools")]
extern crate quickcheck;

extern crate num;

mod aabb;
use aabb::Aabb;
use std::fmt::Debug;

fn max<T: num::Float>(v1: T, v2: T) -> T {
	if v2.is_nan() || (v2 > v1) { v2 }
	else { v1 }
}

fn min<T: num::Float>(v1: T, v2: T) -> T {
	if v2.is_nan() || (v2 < v1) { v2 }
	else { v1 }
}

type Aabb_ = ((f32, f32, f32), (f32, f32, f32));

/// An easily copyable type that represents a value inside the tree
#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd)]
pub struct Proxy {
	v: usize
}

impl Proxy {
	fn empty() -> Proxy {
		Proxy{v: 0}
	}
	fn new(v: usize) -> Proxy {
		Proxy{v: v}
	}
}

/// Defines what kind of relationship the node has with other linked nodes
#[derive(Clone, Debug)]
enum NodeLink {
	/// Nodes with a Parent are part of the tree structure
	Parent(Option<Proxy>),
	/// Nodes with a Next link are nodes that were freed, they point to
	/// the next free node
	Next(Option<Proxy>)
}

/// Defines the status of the node, since a node is either a Leaf or a Parent.
/// These values are never wrong, if a node is a Parent, both child are guaranteed to exist
/// and to be valid, otherwise the node is a Leaf and the data is accessible.
enum NodeStatus<T> {
	Parent(Proxy, Proxy),
	Leaf(T)
}

impl<T> Clone for NodeStatus<T>
where T: Clone {
	fn clone(&self) -> NodeStatus<T> {
		match *self {
			NodeStatus::Parent(ref c1, ref c2) => NodeStatus::Parent(*c1, *c2),
			NodeStatus::Leaf(ref v) => NodeStatus::Leaf(v.clone())
		}
	}
}

impl<T> Debug for NodeStatus<T> where T: Debug {
	fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
		match *self {
			NodeStatus::Parent(ref c1, ref c2) => write!(f, "Parent({}, {})", c1.v, c2.v),
			NodeStatus::Leaf(ref t) => write!(f, "Leaf({:?})", t)
		}
	}
}

struct Node<T>  {
	height: usize,
	aabb: Aabb_,
	status: NodeStatus<T>,
	link: NodeLink,
}

impl<T> Debug for Node<T>
	where T: Debug {
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			write!(f, "Node{{ height: {}, aabb: {:?}, status: {:?}, link: {:?} }}", self.height, self.aabb, self.status, self.link)
		}
}

impl<T> Clone for Node<T>
where T: Clone {
	fn clone(&self) -> Node<T> {
		Node {
			height: self.height,
			aabb: self.aabb,
			status: self.status.clone(),
			link: self.link.clone()
		}
	}
}

impl<T> Node<T> {
	fn new() -> Self {
		Node {
			height: 0,
			aabb: ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
			status: NodeStatus::Parent(Proxy::empty(), Proxy::empty()),
			link: NodeLink::Next(None)
		}
	}
	fn is_leaf(&self) -> bool {
		if let NodeStatus::Leaf(_) = self.status {
			true
		}
		else {
			false
		}
	}
}

/// A dynamic spatial tree. Data is arranged in a binary tree and allows AABB,
/// frustum and raycast queries. The proxies returned from the various methods
/// are leaves in the tree.
///
/// #Panics
///
/// AABB operations are checked for validity and will panic if `aabb.is_valid()`
/// returns false.
#[derive(Clone)]
pub struct AabbTree<T> {
	root: Option<Proxy>,
	nodes: Vec<Node<T>>,
	free_list: Option<Proxy>,
}

impl<T> Debug for AabbTree<T>
	where T: Debug {
	fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
		fn postorder<T: Debug>(f: &mut std::fmt::Formatter, tree: &AabbTree<T>, node: &Node<T>, ident: i32, count: i32) {
			for _ in 0..ident { print!(" "); }
			match node.status {
				NodeStatus::Parent(c1, c2) => {
					let _ = writeln!(f, "> {:?}", node.aabb);
					postorder(f, tree, &tree.nodes[c1.v], ident + 4, count + 1);
					postorder(f, tree, &tree.nodes[c2.v], ident + 4, count + 1);
				}
				NodeStatus::Leaf(ref l) => {
					let _ = writeln!(f, "{:?} {:?}", l, node.aabb);
				}
			}
		}
		postorder(f, self, &self.nodes[self.root.unwrap().v], 0, 0);
		write!(f, "")
	}
}

impl<T> AabbTree<T> {
	/// Returns a new AABB
	#[inline]
	pub fn new() -> Self {
		AabbTree {
			root: None,
			nodes: Vec::new(),
			free_list: None
		}
	}

	fn extended_aabb(aabb: &Aabb_, extension: f32) -> Aabb_ {
		let min = (aabb.min().0 - extension, aabb.min().1 - extension, aabb.min().2 - extension);
		let max = (aabb.max().0 + extension, aabb.max().1 + extension, aabb.max().2 + extension);

		(min, max)
	}

	fn combined_aabb(a1: &Aabb_, a2: &Aabb_) -> Aabb_ {
		(
			(min(a1.min().0, a2.min().0), min(a1.min().1, a2.min().1), min(a1.min().2, a2.min().2)),
			(max(a1.max().0, a2.max().0), max(a1.max().1, a2.max().1), max(a1.max().2, a2.max().2))
		)
	}

	fn allocate_node(&mut self) -> Proxy {
		self.free_list.or_else(
			||{
				let node = Node::new();
				self.nodes.push(node);
				Some(Proxy::new(self.nodes.len() - 1))
			}
		).and_then(
			|proxy|{
				let node = &self.nodes[proxy.v];
				self.free_list = match node.link {
					NodeLink::Next(next) => next,
					_ => unreachable!("Node link should be Next")
				};

				Some(proxy)
			}
		).expect("node allocation")
	}

	fn free_node(&mut self, proxy: Proxy) {
		assert!(proxy.v < self.nodes.len());

		let mut nodes = &mut self.nodes;
		let free_list = self.free_list;

		self.free_list = nodes.get_mut(proxy.v).and_then(|node|{
			node.link = NodeLink::Next(free_list);

			Some(proxy)
		});

		assert!(self.free_list.is_some());
	}

	fn balance2(&mut self, i_a: Proxy, i_s0: Proxy, i_s1: Proxy, swap_child: bool) -> Proxy {
		let nodes_len = self.nodes.len();

		let (i_f, i_g) = match self.nodes[i_s0.v].status {
			NodeStatus::Parent(c1, c2) => (c1, c2),
			_ => unreachable!("node to be balanced has no children")
		};

		assert!(i_f.v < nodes_len && i_g.v < nodes_len);


		self.nodes[i_s0.v].status = NodeStatus::Parent(i_a, i_g);
		self.nodes[i_s0.v].link = self.nodes[i_a.v].link.clone();
		self.nodes[i_a.v].link = NodeLink::Parent(Some(i_s0));

		assert!(!self.nodes[i_s0.v].is_leaf());

		match self.nodes[i_s0.v].link {
			NodeLink::Parent(Some(p)) => {
				self.nodes[p.v].status = match self.nodes[p.v].status {
					NodeStatus::Parent(c1, c2) => {
						if c1 == i_a {
							NodeStatus::Parent(i_s0, c2)
						}
						else {
							assert!(c2 == i_a);
							NodeStatus::Parent(c1, i_s0)
						}
					}
					_ => unreachable!("parent has no children")
				};
			}
			NodeLink::Parent(None) => {
				self.root = Some(i_s0);
			}
			_ => unreachable!("node to be balanced is not a parent")
		}

		let (i_t1, i_t2) = if self.nodes[i_f.v].height > self.nodes[i_g.v].height {
			(i_f, i_g)
		}
		else {
			(i_g, i_f)
		};

		if let NodeStatus::Parent(c1, _) = self.nodes[i_s0.v].status {
			self.nodes[i_s0.v].status = NodeStatus::Parent(c1, i_t1);
		}
		else { unreachable!("node to be balanced is not a parent") }
		if let NodeStatus::Parent(c1, c2) = self.nodes[i_a.v].status {
			if swap_child {
				self.nodes[i_a.v].status = NodeStatus::Parent(i_t2, c2);
			}
			else {
				self.nodes[i_a.v].status = NodeStatus::Parent(c1, i_t2);
			}
		}
		else { unreachable!("node to be balanced is not a parent") };
		self.nodes[i_t2.v].link = NodeLink::Parent(Some(i_a));

		let (temp_aabb1, temp_aabb2) = (self.nodes[i_s1.v].aabb, self.nodes[i_t2.v].aabb);
		self.nodes[i_a.v].aabb = Self::combined_aabb(&temp_aabb1, &temp_aabb2);
		let (temp_aabb1, temp_aabb2) = (self.nodes[i_a.v].aabb, self.nodes[i_t1.v].aabb);
		self.nodes[i_s0.v].aabb = Self::combined_aabb(&temp_aabb1, &temp_aabb2);

		self.nodes[i_a.v].height = 1 + std::cmp::max(self.nodes[i_s1.v].height, self.nodes[i_t2.v].height);//max(, );
		self.nodes[i_s0.v].height = 1 + std::cmp::max(self.nodes[i_a.v].height, self.nodes[i_t1.v].height);

		i_s0
	}

	fn balance(&mut self, i_a: Proxy) -> Proxy {
		assert!(i_a.v < self.nodes.len());

		let nodes_len = self.nodes.len();

		if self.nodes[i_a.v].is_leaf() || self.nodes[i_a.v].height < 2 {
			return i_a
		}

		let (i_b, i_c) = match self.nodes[i_a.v].status {
			NodeStatus::Parent(c1, c2) => (c1, c2),
			_ => unreachable!("node to be balanced has no children")
		};

		assert!(i_b.v < nodes_len && i_c.v < nodes_len);

		let higher_c = self.nodes[i_c.v].height > self.nodes[i_b.v].height;
		let higher_b = self.nodes[i_b.v].height > self.nodes[i_c.v].height;

		if higher_c && (self.nodes[i_c.v].height - self.nodes[i_b.v].height > 1) {
			return self.balance2(i_a, i_c, i_b, false);
		}
		else if higher_b && (self.nodes[i_b.v].height - self.nodes[i_c.v].height > 1) {
			return self.balance2(i_a, i_b, i_c, true);
		}

		i_a
	}

	fn insert_leaf(&mut self, leaf: Proxy) {
		if self.root.is_none() {
			self.root = Some(leaf);
			self.nodes[leaf.v].link = NodeLink::Parent(None);
			return
		}

		let leaf_aabb = self.nodes[leaf.v].aabb;
		let mut index = self.root.expect("root index");

		while !self.nodes[index.v].is_leaf() {
			let (child1, child2) = match self.nodes[index.v].status {
				NodeStatus::Parent(c1, c2) => (c1, c2),
				_ => unreachable!("Node status is not Parent")
			};

			let area = self.nodes[index.v].aabb.perimeter();
			let combined_aabb = Self::combined_aabb(&self.nodes[index.v].aabb, &leaf_aabb);
			let combined_area = combined_aabb.perimeter();

			let cost = 2.0f32 * combined_area;
			let inheritance_cost = 2.0f32 * (combined_area - area);

			let cost1 = if self.nodes[child1.v].is_leaf() {
				let aabb = Self::combined_aabb(&leaf_aabb, &self.nodes[child1.v].aabb);
				aabb.perimeter() + inheritance_cost
			}
			else {
				let aabb = Self::combined_aabb(&leaf_aabb, &self.nodes[child1.v].aabb);
				let old_area = self.nodes[child1.v].aabb.perimeter();
				let new_area = aabb.perimeter();
				(new_area - old_area) + inheritance_cost
			};

			let cost2 = if self.nodes[child2.v].is_leaf() {
				let aabb = Self::combined_aabb(&leaf_aabb, &self.nodes[child2.v].aabb);
				aabb.perimeter() + inheritance_cost
			}
			else {
				let aabb = Self::combined_aabb(&leaf_aabb, &self.nodes[child2.v].aabb);
				let old_area = self.nodes[child2.v].aabb.perimeter();
				let new_area = aabb.perimeter();
				(new_area - old_area) + inheritance_cost
			};

			if cost < cost1 && cost < cost2 {
				break
			}
			if cost1 < cost2 {
				index = child1;
			}
			else {
				index = child2;
			}
		}

		let sibling = index;

		let old_parent = match self.nodes[sibling.v].link {
			NodeLink::Parent(p) => p,
			_ => unreachable!("sibling status is not parent")
		};
		let new_parent = self.allocate_node();
		self.nodes[new_parent.v].link = NodeLink::Parent(old_parent);

		let temp_aabb = self.nodes[sibling.v].aabb;
		self.nodes[new_parent.v].aabb = Self::combined_aabb(&leaf_aabb, &temp_aabb);
		self.nodes[new_parent.v].height = self.nodes[sibling.v].height + 1;

		match old_parent {
			Some(old_parent) => {
				let (child1, child2) = match self.nodes[old_parent.v].status {
					NodeStatus::Parent(c1, c2) => (c1, c2),
					_ => unreachable!("old parent doesn't have children")
				};

				if child1 == sibling {
					self.nodes[old_parent.v].status = NodeStatus::Parent(new_parent, child2);
				}
				else {
					self.nodes[old_parent.v].status = NodeStatus::Parent(child1, new_parent);
				}
			},
			None => {
				self.root = Some(new_parent);
			}
		}

		self.nodes[new_parent.v].status = NodeStatus::Parent(sibling, leaf);
		self.nodes[sibling.v].link = NodeLink::Parent(Some(new_parent));
		self.nodes[leaf.v].link = NodeLink::Parent(Some(new_parent));

		let mut index = match self.nodes[leaf.v].link {
			NodeLink::Parent(p) => p,
			_ => unreachable!("leaf has wrong link")
		};
		while let Some(mut i) = index {
			i = self.balance(i);

			let (child1, child2) = match self.nodes[i.v].status {
				NodeStatus::Parent(c1, c2) => (c1, c2),
				_ => unreachable!("parent has no children")
			};

			self.nodes[i.v].height = 1 + std::cmp::max(self.nodes[child1.v].height, self.nodes[child2.v].height);

			let aabb1 = self.nodes[child1.v].aabb;
			let aabb2 = self.nodes[child2.v].aabb;
			self.nodes[i.v].aabb = Self::combined_aabb(&aabb1, &aabb2);

			index = match self.nodes[i.v].link {
				NodeLink::Parent(p) => p,
				_ => unreachable!("child node has wrong link")
			};
		}
	}

	fn remove_leaf(&mut self, leaf: Proxy) {
		if Some(leaf) == self.root {
			self.root = None;
			return;
		}

		let parent = match self.nodes[leaf.v].link {
			NodeLink::Parent(Some(p)) => p,
			_ => unreachable!("leaf to be removed has no parent")
		};
		let grand_parent = match self.nodes[parent.v].link {
			NodeLink::Parent(p) => p,
			_ => unreachable!("leaf's grandparent is a leaf")
		};

		let sibling = match self.nodes[parent.v].status {
			NodeStatus::Parent(c1, c2) => {
				if c1 == leaf {
					c2
				}
				else {
					c1
				}
			}
			_ => unreachable!("parent has no children")
		};

		match grand_parent {
			Some(grand_parent) => {
				self.nodes[grand_parent.v].status = match self.nodes[grand_parent.v].status {
					NodeStatus::Parent(c1, c2) => {
						if c1 == parent {
							NodeStatus::Parent(sibling, c2)
						}
						else {
							NodeStatus::Parent(c1, sibling)
						}
					}
					_ => unreachable!("grand_parent is not a parent")
				};
				self.nodes[sibling.v].link = NodeLink::Parent(Some(grand_parent));
				self.free_node(parent);

				let mut index = Some(grand_parent);
				while let Some(mut i) = index {
					i = self.balance(i);

					let (child1, child2) = match self.nodes[i.v].status {
						NodeStatus::Parent(c1, c2) => (c1, c2),
						_ => unreachable!("node has no children")
					};

					let (temp_aabb1, temp_aabb2) = (self.nodes[child1.v].aabb, self.nodes[child2.v].aabb);
					self.nodes[i.v].aabb = Self::combined_aabb(&temp_aabb1, &temp_aabb2);
					self.nodes[i.v].height = 1 + std::cmp::max(self.nodes[child1.v].height, self.nodes[child2.v].height);

					index = match self.nodes[i.v].link {
						NodeLink::Parent(p) => p,
						_ => unreachable!("node has no parent")
					};
				}
			}
			None => {
				self.root = Some(sibling);
				self.nodes[sibling.v].link = NodeLink::Parent(None);
				self.free_node(parent);
			}
		}
	}
	/// Creates a new node in the tree.

	pub fn create_proxy(&mut self, mut aabb: Aabb_, user_data: T) -> Proxy {
		let proxy = self.allocate_node();

		self.nodes.get_mut(proxy.v).and_then(|node|{
			aabb = Self::extended_aabb(&aabb, 0.1f32);

			node.aabb = aabb;
			node.status = NodeStatus::Leaf(user_data);
			//node.height = 0;
			assert!(node.height == 0);

			Some(0)
		});

		self.insert_leaf(proxy);
		proxy
	}
	/// Destroy a node in the tree. Does not deallocate memory, nodes can be reutilized
	/// on later `create_proxy` calls.
	/// # Panics
	///
	/// Panics if the proxy_id is not valid (either was destroyed already or is not a leaf).
	/// Use proxies returned by `create_proxy` to avoid problems.

	pub fn destroy_proxy(&mut self, proxy: Proxy) {
		assert!(proxy.v < self.nodes.len());
		assert!(self.nodes[proxy.v].is_leaf());

		self.remove_leaf(proxy);
		self.free_node(proxy);
	}
	/// Changes the AABB of a node, might trigger a tree rebalance if the new AABB
	/// doesn't fit within the node's parent AABB.
	///
	/// # Panics
	///
	/// Panics if the proxy_id is not valid (either was destroyed already or is not a leaf).
	/// Use proxies returned by `create_proxy` to avoid problems.

	pub fn set_aabb(&mut self, proxy: Proxy, new_aabb: &Aabb_) {
		assert!(proxy.v < self.nodes.len());
		assert!(self.nodes[proxy.v].is_leaf());

		let new_aabb = Self::extended_aabb(new_aabb, 0.1f32);
		self.nodes[proxy.v].aabb = new_aabb;

		match self.nodes[proxy.v].link {
			NodeLink::Parent(Some(parent_proxy)) => {
				if self.nodes[parent_proxy.v].aabb.contains(&new_aabb) {
					return;
				}
			}
			NodeLink::Parent(None) => {}
			NodeLink::Next(_) => panic!("node is not a part of the tree hierarchy")
		}

		self.remove_leaf(proxy);
		self.insert_leaf(proxy);
	}
	/// Returns a reference to the user data associated with the `proxy_id`. Returns
	/// `None` if the `proxy_id` is invalid.

	pub fn user_data(&self, proxy: Proxy) -> Option<&T> {
		self.nodes.get(proxy.v).and_then(|node| {
			match node.status {
				NodeStatus::Leaf(ref t) => Some(t),
				_ => None
			}
		})
	}
	/// Returns a mutable reference to the user data associated with the `proxy_id`. Returns
	/// `None` if the `proxy_id` is invalid.

	pub fn user_data_mut(&mut self, proxy: Proxy) -> Option<&mut T> {
		self.nodes.get_mut(proxy.v).and_then(|mut node| {
			match node.status {
				NodeStatus::Leaf(ref mut t) => Some(t),
				_ => None
			}
		})
	}
	/// Searchs the AABB for every node that overlaps with `aabb` and calls the `callback`
	/// with the node's proxy id as a parameter. The callback returns a boolean value
	/// indicating wether the query should continue or not. The method is a specialized version
	/// of `AabbTree::query`, where `search` is defined as `|other_aabb| other_aabb.overlaps(aabb)`

	pub fn query_aabb<F: Fn(Proxy) -> bool>(&self, aabb: &Aabb_, callback: F) {
		self.query(|other_aabb| other_aabb.overlaps(aabb), callback);
	}
	/// Searchs the AABB using a generic `search` function. `search` receives the current
	/// AABB being analyzed and should return true if that AABB is within your search
	/// constraints. Once a leaf node is found and `search` returns true for that node,
	/// `callback` will be called with the node's proxy id as a parameter. The callback
	/// returns a boolean value indicating wether the query should continue or not.

	pub fn query<F, S>(&self, search: S, callback: F)
	where F: Fn(Proxy) -> bool, S: Fn(&Aabb_) -> bool {

		let mut stack = Vec::new();

		if let Some(root) = self.root {
			stack.push(root);
		}

		while let Some(node_id) = stack.pop() {
			if search(&self.nodes[node_id.v].aabb) {
				match self.nodes[node_id.v].status {
					NodeStatus::Leaf(_) => {
						if !callback(node_id) {
							break;
						}
					}
					NodeStatus::Parent(c1, c2) => {
						stack.push(c1);
						stack.push(c2);
					}
				}
			}
		}
	}
}


#[cfg(feature="test_tools")]
#[cfg(test)]
mod tests {
	use super::{AabbTree, NodeStatus, NodeLink, Proxy};
	use super::aabb::Aabb;
	use quickcheck;

	type Vec3 = (f32, f32, f32);
	type Aabb_ = (Vec3, Vec3);

	/// Returns either a random `AabbTree` or a `quickcheck::TestResult` if the tree is empty.
	fn random_tree(aabbs: Vec<Aabb_>) -> Result<AabbTree<i32>, quickcheck::TestResult> {
		let mut tree = AabbTree::new();

		for (i, aabb) in aabbs.iter().filter(|aabb| aabb.is_valid()).enumerate() {
			tree.create_proxy(*aabb, i as i32);
		}

		if tree.root.is_none() { Err(quickcheck::TestResult::discard()) }
		else { Ok(tree) }
	}

	/// This makes sure some constraints are respected in random trees. The constraints are:
	/// * Structural validation
	/// Validates every node, checking if the status and links are correct.
	/// * Metrics validation
	/// Makes sure the combined AABB of two children in equal to the parent's AABB and the height
	/// is correct.
	#[test]
	fn validation() {
		fn tree_validation(aabbs: Vec<Aabb_>) -> quickcheck::TestResult {
			let tree = match random_tree(aabbs) {
				Ok(tree) => tree,
				Err(res) => return res
			};

			fn test_structure<T>(t: &AabbTree<T>, n: Proxy) -> bool {
				let node = &t.nodes[n.v];

				if t.root.unwrap().v == n.v {
					if let NodeLink::Parent(None) = node.link {}
					else {
						return false
					}
				}

				match node.status {
					NodeStatus::Leaf(_) => node.height == 0,
					NodeStatus::Parent(ref c1, ref c2) => {
						match (&t.nodes[c1.v].link, &t.nodes[c2.v].link) {
							(&NodeLink::Parent(Some(p1)), &NodeLink::Parent(Some(p2))) => {
								(p1 == n) && (p2 == n) && test_structure(t, *c1) && test_structure(t, *c2)
							}
							_ => false
						}
					}
				}
			}

			fn test_metrics<T>(t: &AabbTree<T>, n: Proxy) -> bool {
				let node = &t.nodes[n.v];

				if t.root.unwrap().v == n.v {
					if let NodeLink::Parent(None) = node.link {}
					else {
						return false
					}
				}

				match node.status {
					NodeStatus::Leaf(_) => node.height == 0,
					NodeStatus::Parent(ref c1, ref c2) => {
						let height1 = t.nodes[c1.v].height;
						let height2 = t.nodes[c2.v].height;

						if (1 + ::std::cmp::max(height1, height2)) != node.height {
							false
						}
						else {
							let c_aabb = AabbTree::<i32>::combined_aabb(&t.nodes[c1.v].aabb, &t.nodes[c2.v].aabb);

							c_aabb.min() == node.aabb.min() &&
							c_aabb.max() == node.aabb.max() &&
							test_metrics(t, *c1) &&
							test_metrics(t, *c2)
						}
					}
				}
			}

			quickcheck::TestResult::from_bool(
				test_structure(&tree, tree.root.expect("root")) &&
				test_metrics(&tree, tree.root.expect("root"))
			)
		}

		quickcheck::quickcheck(tree_validation as fn(Vec<Aabb_>) -> quickcheck::TestResult);
	}

	/// Checks if every proxy has a status of `Leaf`
	#[test]
	fn inserted_is_leaf() {
		fn tree_insertion(n: i32) -> bool {
			let mut tree : AabbTree<i32> = AabbTree::new();

			let mut proxies = Vec::new();
			for i in 0..n {
				proxies.push(tree.create_proxy(((-1.0,-1.0,-1.0), (1.0,1.0,1.0)), i));
			}

			for (i, v) in proxies.iter().enumerate() {
				if !tree.nodes[v.v].is_leaf() {
					return false;
				}
				match tree.nodes[v.v].status {
					NodeStatus::Leaf(t) => {
						if i as i32 != t {
							return false
						}
					},
					_ => unreachable!()
				}
			}
			true
		}

		quickcheck::quickcheck(tree_insertion as fn(i32) -> bool);
	}

	/// Tests the size of the AABB nodes. Children nodes should always have smaller AABBs than their
	/// parents.
	#[test]
	fn insertion_size() {
		fn tree_insertion(aabbs: Vec<Aabb_>) -> quickcheck::TestResult {
			let tree = match random_tree(aabbs) {
				Ok(tree) => tree,
				Err(res) => return res
			};

			fn test_aabb_sizes<T: ::std::fmt::Debug>(t: &AabbTree<T>, n: Proxy, mut parent_aabb: Option<Aabb_>) -> bool {
				parent_aabb = match parent_aabb {
					None => Some(t.nodes[n.v].aabb),
					Some(aabb) => {
						if aabb.perimeter() < t.nodes[n.v].aabb.perimeter() {
							return false
						}
						if !aabb.overlaps(&t.nodes[n.v].aabb) {
							return false
						}
						if !aabb.contains(&t.nodes[n.v].aabb) {
							return false
						}

						Some(aabb)
					}
				};

				if let NodeStatus::Parent(c1, c2) = t.nodes[n.v].status {
					test_aabb_sizes(t, c1, parent_aabb)
					&& test_aabb_sizes(t, c2, parent_aabb)
				}
				else {
					true
				}
			}

			quickcheck::TestResult::from_bool(test_aabb_sizes(&tree, tree.root.expect("root"), None))
		}

		quickcheck::quickcheck(tree_insertion as fn(Vec<Aabb_>) -> quickcheck::TestResult);
	}
}