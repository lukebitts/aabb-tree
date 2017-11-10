//#![deny(missing_docs)]

//#![cfg_attr(test, deny(warnings))]
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
//! This work is based on Presson's `btDbvt` written for Bullet Physics and
//! Catto's `b2DynamicTree` written for Box2D.

#[cfg(feature = "test_tools")]
extern crate quickcheck;
#[cfg(feature = "test_tools")]
extern crate rand;

extern crate num;
extern crate statrs;
extern crate noisy_float;

mod aabb;
use aabb::Aabb;
use std::fmt::Debug;

fn max<T: num::Float>(v1: T, v2: T) -> T {
    if v2.is_nan() || (v2 > v1) { v2 } else { v1 }
}

fn min<T: num::Float>(v1: T, v2: T) -> T {
    if v2.is_nan() || (v2 < v1) { v2 } else { v1 }
}

/// How the tree represents AABBs internally
pub type MinMaxTuple<P> = ((P, P, P), (P, P, P));

impl<P: num::Float + ::std::fmt::Debug> Aabb for MinMaxTuple<P> {
    type Precision = P;
    fn with_params(
        min: (Self::Precision, Self::Precision, Self::Precision),
        max: (Self::Precision, Self::Precision, Self::Precision),
    ) -> Self
    where
        Self: Sized,
    {
        (min, max)
    }
    fn min(&self) -> (Self::Precision, Self::Precision, Self::Precision) {
        self.0
    }
    fn max(&self) -> (Self::Precision, Self::Precision, Self::Precision) {
        self.1
    }
}

/// An easily copyable type that represents a value inside the tree
#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct Proxy {
    v: usize,
}

impl Proxy {
    fn new(v: usize) -> Proxy {
        Proxy { v: v }
    }
}

/// Defines what kind of relationship the node has with other linked nodes
#[derive(Clone, Debug)]
enum NodeLink {
    /// Nodes with a Parent are part of the tree structure
    Parent(Option<Proxy>),
    /// Nodes with a Next link are nodes that were freed, they point to
    /// the next free node
    Next(Option<Proxy>),
}

/// Defines the status of the node, since a node is either a Leaf or a Parent.
/// These values are never wrong, if a node is a Parent, both child are guaranteed to exist
/// and to be valid, otherwise the node is a Leaf and the data is accessible.
#[derive(Clone, Debug)]
enum NodeStatus<T> {
    Parent(Proxy, Proxy),
    Leaf(Option<T>),
}

#[derive(Clone, Debug)]
struct Node<T, P>
where
    P: num::Float + num::FromPrimitive,
{
    height: usize,
    aabb: MinMaxTuple<P>,
    status: NodeStatus<T>,
    link: NodeLink,
}

impl<T, P> Node<T, P>
where
    P: num::Float + num::FromPrimitive,
{
    fn new(status: NodeStatus<T>) -> Self {
        let z = P::from_f64(0.0).expect("f64 to P");

        Node::<T, P> {
            height: 0,
            aabb: ((z, z, z), (z, z, z)),
            status: status,
            link: NodeLink::Next(None),
        }
    }
    fn is_leaf(&self) -> bool {
        if let NodeStatus::Leaf(_) = self.status {
            true
        } else {
            false
        }
    }
    fn is_parent(&self) -> bool {
        !self.is_leaf()
    }
}

#[derive(Copy, Clone, Eq, PartialEq)]
enum RotateDirection {
    Left,
    Right,
}

/// A dynamic spatial tree. Data is arranged in a binary tree which allows fast positional queries.
/// The proxies returned from the various methods are leaves in the tree.
///
/// #Panics
///
/// AABB operations are checked for validity and will panic in debug mode if `aabb.is_valid()`
/// returns false.
#[derive(Clone)]
pub struct AabbTree<T, P = f32, A = MinMaxTuple<f32>>
where
    P: num::Float + num::FromPrimitive + Debug,
    A: Into<MinMaxTuple<P>> + From<MinMaxTuple<P>> + Clone,
{
    root: Option<Proxy>,
    nodes: Vec<Node<T, P>>,
    free_list: Option<Proxy>,
    _p: ::std::marker::PhantomData<A>,
}

impl<T, P, A> Debug for AabbTree<T, P, A>
where T: Debug,
	  P: num::Float + num::FromPrimitive + Debug,
	  A: Into<MinMaxTuple<P>> + From<MinMaxTuple<P>> + Clone + Debug
{
	fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
		fn postorder<T, P, A>(f: &mut std::fmt::Formatter, tree: &AabbTree<T, P, A>, node: &Node<T, P>, proxy: Proxy, ident: i32, count: i32)
		where T: Debug,
			  P: num::Float + num::FromPrimitive + Debug,
			  A: Into<MinMaxTuple<P>> + From<MinMaxTuple<P>> + Clone + Debug
		{
            if count > 30 { return }
			for _ in 0..ident { print!(" "); }
			match node.status {
				NodeStatus::Parent(c1, c2) => {
					let _ = writeln!(f, "parent> |{}| {:?} {:?}", proxy.v, node.link, node.aabb);
					postorder(f, tree, &tree.nodes[c1.v], c1, ident + 4, count + 1);
					postorder(f, tree, &tree.nodes[c2.v], c2, ident + 4, count + 1);
				}
				NodeStatus::Leaf(ref l) => {
					let _ = writeln!(f, "leaf> {}| {:?} {:?}", proxy.v, l, node.aabb);
				}
			}
		}
        if let Some(root) = self.root {
            postorder(f, self, &self.nodes[root.v], root, 0, 0);
            write!(f, "")
        }
        else {
            Ok(())
        }
	}
}

impl<T, P, A> Default for AabbTree<T, P, A>
where
    P: num::Float + num::FromPrimitive + Debug,
    A: Into<MinMaxTuple<P>> + From<MinMaxTuple<P>> + Clone
{
    fn default() -> Self {
        AabbTree::new()
    }
}

impl<T, P, A> AabbTree<T, P, A>
where
    P: num::Float + num::FromPrimitive + Debug,
    A: Into<MinMaxTuple<P>> + From<MinMaxTuple<P>> + Clone,
{
    /// Returns a new AABB
    #[inline]
    pub fn new() -> Self {
        AabbTree {
            root: None,
            nodes: Vec::new(),
            free_list: None,
            _p: ::std::marker::PhantomData,
        }
    }

    fn extended_aabb(aabb: &MinMaxTuple<P>, extension: P) -> MinMaxTuple<P> {
        use aabb::Aabb;
        let min = (
            aabb.min().0 - extension,
            aabb.min().1 - extension,
            aabb.min().2 - extension,
        );
        let max = (
            aabb.max().0 + extension,
            aabb.max().1 + extension,
            aabb.max().2 + extension,
        );

        (min, max)
    }

    fn combined_aabb(a1: &MinMaxTuple<P>, a2: &MinMaxTuple<P>) -> MinMaxTuple<P> {
        (
            (
                min(a1.min().0, a2.min().0),
                min(a1.min().1, a2.min().1),
                min(a1.min().2, a2.min().2),
            ),
            (
                max(a1.max().0, a2.max().0),
                max(a1.max().1, a2.max().1),
                max(a1.max().2, a2.max().2),
            ),
        )
    }

    fn allocate_node(&mut self, status: NodeStatus<T>) -> Proxy {

        let proxy = self.free_list.unwrap_or_else(|| {
            let node = Node::new(NodeStatus::Leaf(None));
            self.nodes.push(node);
            Proxy::new(self.nodes.len() - 1)
        });

        let mut node = &mut self.nodes[proxy.v];
        node.status = status;
        self.free_list = match node.link {
            NodeLink::Next(next) => next,
            _ => unreachable!("Node link should be Next"),
        };

        proxy
    }

    fn free_node(&mut self, proxy: Proxy) {
        assert!(proxy.v < self.nodes.len());

        let mut nodes = &mut self.nodes;
        let free_list = self.free_list;

        self.free_list = nodes.get_mut(proxy.v).and_then(|node| {
            node.link = NodeLink::Next(free_list);

            Some(proxy)
        });

        assert!(self.free_list.is_some());
    }

    fn rotate(&mut self, parent: Proxy, rotation: RotateDirection) {
        let nodes_len = self.nodes.len();

        let (child1, child2) = match (rotation, &self.nodes[parent.v].status) {
            (RotateDirection::Left, &NodeStatus::Parent(c1, c2)) => (c1, c2),
            (RotateDirection::Right, &NodeStatus::Parent(c1, c2)) => (c2, c1),
            _ => unreachable!("node to rotate is not a parent"),
        };

        let (child1_child_left, child1_child_right) = match self.nodes[child1.v].status {
            NodeStatus::Parent(c1, c2) => (c1, c2),
            _ => unreachable!("node to be balanced has no children"),
        };

        assert!(child1_child_left.v < nodes_len && child1_child_right.v < nodes_len);

        self.nodes[child1.v].status = NodeStatus::Parent(parent, child1_child_right);
        self.nodes[child1.v].link = self.nodes[parent.v].link.clone();
        self.nodes[parent.v].link = NodeLink::Parent(Some(child1));

        assert!(!self.nodes[child1.v].is_leaf());

        match self.nodes[child1.v].link {
            NodeLink::Parent(Some(p)) => {
                self.nodes[p.v].status = match self.nodes[p.v].status {
                    NodeStatus::Parent(c1, c2) => {
                        if c1 == parent {
                            NodeStatus::Parent(child1, c2)
                        } else {
                            assert_eq!(c2, parent);
                            NodeStatus::Parent(c1, child1)
                        }
                    }
                    _ => unreachable!("parent has no children"),
                };
            }
            NodeLink::Parent(None) => {
                self.root = Some(child1);
            }
            _ => unreachable!("node to be balanced is not a parent"),
        }

        let (i_t1, i_t2) =
            if self.nodes[child1_child_left.v].height > self.nodes[child1_child_right.v].height {
                (child1_child_left, child1_child_right)
            } else {
                (child1_child_right, child1_child_left)
            };

        self.nodes[child1.v].status = NodeStatus::Parent(parent, i_t1);

        if let NodeStatus::Parent(c1, c2) = self.nodes[parent.v].status {
            match rotation {
                RotateDirection::Left => self.nodes[parent.v].status = NodeStatus::Parent(i_t2, c2),
                RotateDirection::Right => {
                    self.nodes[parent.v].status = NodeStatus::Parent(c1, i_t2)
                }
            }
        } else {
            unreachable!("node to be balanced is not a parent")
        };
        self.nodes[i_t2.v].link = NodeLink::Parent(Some(parent));

        let (temp_aabb1, temp_aabb2) = (
            self.nodes[child2.v].aabb.into(),
            self.nodes[i_t2.v].aabb.into(),
        );
        self.nodes[parent.v].aabb = Self::combined_aabb(&temp_aabb1, &temp_aabb2);
        let (temp_aabb1, temp_aabb2) = (
            self.nodes[parent.v].aabb.into(),
            self.nodes[i_t1.v].aabb.into(),
        );
        self.nodes[child1.v].aabb = Self::combined_aabb(&temp_aabb1, &temp_aabb2);

        self.nodes[parent.v].height = 1 +
            std::cmp::max(self.nodes[child2.v].height, self.nodes[i_t2.v].height);
        self.nodes[child1.v].height = 1 +
            std::cmp::max(self.nodes[parent.v].height, self.nodes[i_t1.v].height);
    }

    fn balance(&mut self, proxy: Proxy) -> Proxy {
        assert!(proxy.v < self.nodes.len());

        let nodes_len = self.nodes.len();

        if self.nodes[proxy.v].is_leaf() || self.nodes[proxy.v].height < 2 {
            return proxy;
        }

        let (child_left, child_right) = match self.nodes[proxy.v].status {
            NodeStatus::Parent(c1, c2) => (c1, c2),
            _ => unreachable!("node to be balanced has no children"),
        };

        assert!(child_left.v < nodes_len && child_right.v < nodes_len);


        let higher_left = self.nodes[child_left.v].height > self.nodes[child_right.v].height;
        let higher_right = self.nodes[child_right.v].height > self.nodes[child_left.v].height;

        if higher_right &&
            (self.nodes[child_right.v].height - self.nodes[child_left.v].height > 1)
        {
            self.rotate(proxy, RotateDirection::Right);
            return child_right;
        } else if higher_left &&
                   (self.nodes[child_left.v].height - self.nodes[child_right.v].height > 1)
        {
            self.rotate(proxy, RotateDirection::Left);
            return child_left;
        }

        proxy
    }

    fn insert_leaf(&mut self, leaf: Proxy) {

        if self.root.is_none() {
            self.root = Some(leaf);
            self.nodes[leaf.v].link = NodeLink::Parent(None);
            return;
        }

        let leaf_aabb = self.nodes[leaf.v].aabb;
        let mut index = self.root.expect("root index");

        let mut count = 0;

        while !self.nodes[index.v].is_leaf() {            
            count += 1;

            if count > 30 {
                panic!();
            }

            let (child1, child2) = match self.nodes[index.v].status {
                NodeStatus::Parent(c1, c2) => (c1, c2),
                _ => unreachable!("Node status is not Parent"),
            };

            let area = self.nodes[index.v].aabb.perimeter();
            let combined_aabb = Self::combined_aabb(&self.nodes[index.v].aabb, &leaf_aabb);
            let combined_area = combined_aabb.perimeter();

            let two = P::from_f64(2.0).expect("f64 to P");
            let cost = two * combined_area;
            let inheritance_cost = two * (combined_area - area);

            let cost1 = if self.nodes[child1.v].is_leaf() {
                let aabb = Self::combined_aabb(&leaf_aabb, &self.nodes[child1.v].aabb);
                aabb.perimeter() + inheritance_cost
            } else {
                let aabb = Self::combined_aabb(&leaf_aabb, &self.nodes[child1.v].aabb);
                let old_area = self.nodes[child1.v].aabb.perimeter();
                let new_area = aabb.perimeter();
                (new_area - old_area) + inheritance_cost
            };

            let cost2 = if self.nodes[child2.v].is_leaf() {
                let aabb = Self::combined_aabb(&leaf_aabb, &self.nodes[child2.v].aabb);
                aabb.perimeter() + inheritance_cost
            } else {
                let aabb = Self::combined_aabb(&leaf_aabb, &self.nodes[child2.v].aabb);
                let old_area = self.nodes[child2.v].aabb.perimeter();
                let new_area = aabb.perimeter();
                (new_area - old_area) + inheritance_cost
            };

            if cost < cost1 && cost < cost2 {
                break;
            }
            if cost1 < cost2 {
                index = child1;
            } else {
                index = child2;
            }
        }

        let sibling = index;

        let old_parent = match self.nodes[sibling.v].link {
            NodeLink::Parent(p) => p,
            _ => unreachable!("sibling status is not parent"),
        };

        //let new_parent = self.allocate_node();
        //self.nodes[new_parent.v].link = NodeLink::Parent(old_parent);

        //let temp_aabb = self.nodes[sibling.v].aabb;
        //self.nodes[new_parent.v].aabb = Self::combined_aabb(&leaf_aabb, &temp_aabb);
        //self.nodes[new_parent.v].height = self.nodes[sibling.v].height + 1;

        let new_parent = self.allocate_node(NodeStatus::Parent(sibling, leaf));
        self.nodes[new_parent.v].link = NodeLink::Parent(old_parent);

        let temp_aabb = self.nodes[sibling.v].aabb;
        self.nodes[new_parent.v].aabb = Self::combined_aabb(&leaf_aabb, &temp_aabb);
        self.nodes[new_parent.v].height = self.nodes[sibling.v].height + 1;

        match old_parent {
            Some(old_parent) => {
                let (child1, child2) = match self.nodes[old_parent.v].status {
                    NodeStatus::Parent(c1, c2) => (c1, c2),
                    _ => unreachable!("old parent doesn't have children"),
                };

                if child1 == sibling {
                    self.nodes[old_parent.v].status = NodeStatus::Parent(new_parent, child2);
                } else {
                    self.nodes[old_parent.v].status = NodeStatus::Parent(child1, new_parent);
                }
            }
            None => {
                self.root = Some(new_parent);
            }
        }

        //self.nodes[new_parent.v].status = NodeStatus::Parent(sibling, leaf);
        self.nodes[sibling.v].link = NodeLink::Parent(Some(new_parent));
        self.nodes[leaf.v].link = NodeLink::Parent(Some(new_parent));

        let mut index = match self.nodes[leaf.v].link {
            NodeLink::Parent(p) => p,
            _ => unreachable!("leaf has wrong link"),
        };
        while let Some(mut i) = index {
            i = self.balance(i);

            let (child1, child2) = match self.nodes[i.v].status {
                NodeStatus::Parent(c1, c2) => (c1, c2),
                _ => unreachable!("parent has no children"),
            };

            self.nodes[i.v].height = 1 +
                std::cmp::max(self.nodes[child1.v].height, self.nodes[child2.v].height);

            let aabb1 = self.nodes[child1.v].aabb;
            let aabb2 = self.nodes[child2.v].aabb;
            self.nodes[i.v].aabb = Self::combined_aabb(&aabb1, &aabb2);

            index = match self.nodes[i.v].link {
                NodeLink::Parent(p) => p,
                _ => unreachable!("child node has wrong link"),
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
            _ => unreachable!("leaf to be removed has no parent"),
        };
        let grand_parent = match self.nodes[parent.v].link {
            NodeLink::Parent(p) => p,
            _ => unreachable!("leaf's grandparent is a leaf"),
        };

        let sibling = match self.nodes[parent.v].status {
            NodeStatus::Parent(c1, c2) => if c1 == leaf { c2 } else { c1 },
            _ => unreachable!("parent has no children"),
        };

        match grand_parent {
            Some(grand_parent) => {
                self.nodes[grand_parent.v].status = match self.nodes[grand_parent.v].status {
                    NodeStatus::Parent(c1, c2) => {
                        if c1 == parent {
                            NodeStatus::Parent(sibling, c2)
                        } else {
                            NodeStatus::Parent(c1, sibling)
                        }
                    }
                    _ => unreachable!("grand_parent is not a parent"),
                };
                self.nodes[sibling.v].link = NodeLink::Parent(Some(grand_parent));
                self.free_node(parent);

                let mut index = Some(grand_parent);
                while let Some(mut i) = index {
                    i = self.balance(i);

                    let (child1, child2) = match self.nodes[i.v].status {
                        NodeStatus::Parent(c1, c2) => (c1, c2),
                        _ => unreachable!("node has no children"),
                    };

                    let (temp_aabb1, temp_aabb2) =
                        (self.nodes[child1.v].aabb, self.nodes[child2.v].aabb);
                    self.nodes[i.v].aabb = Self::combined_aabb(&temp_aabb1, &temp_aabb2);
                    self.nodes[i.v].height = 1 +
                        std::cmp::max(self.nodes[child1.v].height, self.nodes[child2.v].height);

                    index = match self.nodes[i.v].link {
                        NodeLink::Parent(p) => p,
                        _ => unreachable!("node has no parent"),
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
    pub fn create_proxy(&mut self, aabb: A, user_data: T) -> Proxy {
        let proxy = self.allocate_node(NodeStatus::Leaf(Some(user_data)));

        if let Some(node) = self.nodes.get_mut(proxy.v) {
            let aabb = Self::extended_aabb(&aabb.into(), P::from_f64(0.1).expect("f64 to P"));

            node.aabb = aabb;
            //node.status = NodeStatus::Leaf(user_data);
            //node.height = 0;
            //assert!(match node.link { NodeLink::Next(None) => true, _ => false });
            assert_eq!(node.height, 0);
        }

        self.insert_leaf(proxy);
        proxy
    }

    /// Destroy a node in the tree. Does not deallocate memory, nodes can be reutilized
    /// on later `create_proxy` calls.
    /// # Panics
    ///
    /// Panics if the proxy_id is not valid (either was destroyed already or is not a leaf).
    /// Use proxies returned by `create_proxy` to avoid problems.
    pub fn destroy_proxy(&mut self, proxy: Proxy) -> T {
        assert!(proxy.v < self.nodes.len());
        assert!(self.nodes[proxy.v].is_leaf());

        self.remove_leaf(proxy);
        self.free_node(proxy);
        
        let ret = std::mem::replace(&mut self.nodes[proxy.v].status, NodeStatus::Leaf(None));

        match ret {
            NodeStatus::Leaf(Some(v)) => v,
            NodeStatus::Leaf(None) => panic!("empty leaf"),
            _ => unreachable!("proxy is not a leaf")
        }
    }

    /// Changes the AABB of a node, might trigger a tree rebalance if the new AABB
    /// doesn't fit within the node's parent AABB.
    ///
    /// # Panics
    ///
    /// Panics if the proxy_id is not valid (either was destroyed already or is not a leaf).
    /// Use proxies returned by `create_proxy` to avoid problems.
    pub fn set_aabb(&mut self, proxy: Proxy, new_aabb: &A) {
        assert!(proxy.v < self.nodes.len());
        assert!(self.nodes[proxy.v].is_leaf());

        /*let new_aabb = Self::extended_aabb(
            &(*new_aabb).clone().into(),
            P::from_f64(0.1).expect("f64 to P"),
        );
        self.nodes[proxy.v].aabb = new_aabb;

        match self.nodes[proxy.v].link {
            NodeLink::Parent(Some(parent_proxy)) => {
                if self.nodes[parent_proxy.v].aabb.contains(&new_aabb) {
                    return;
                }
            }
            NodeLink::Parent(None) => return,
            NodeLink::Next(_) => panic!("node is not a part of the tree hierarchy"),
        }

        println!("set aabb {}", proxy.v);*/

        //self.remove_leaf(proxy);
        let x = self.destroy_proxy(proxy);
        self.create_proxy(new_aabb.clone(), x);
        //self.insert_leaf(proxy);
    }

    /// Returns a reference to the user data associated with the `proxy_id`. Returns
    /// `None` if the `proxy_id` is invalid.
    pub fn user_data(&self, proxy: Proxy) -> Option<&T> {
        self.nodes.get(proxy.v).and_then(|node| match node.status {
            NodeStatus::Leaf(Some(ref t)) => Some(t),
            _ => None,
        })
    }

    /// Returns a mutable reference to the user data associated with the `proxy_id`. Returns
    /// `None` if the `proxy_id` is invalid.
    pub fn user_data_mut(&mut self, proxy: Proxy) -> Option<&mut T> {
        self.nodes.get_mut(proxy.v).and_then(
            |mut node| match node.status {
                NodeStatus::Leaf(Some(ref mut t)) => Some(t),
                _ => None,
            },
        )
    }

    /// Searchs the tree for every node that overlaps with `aabb` and calls the `callback`
    /// with the node's proxy id as a parameter. The callback returns a boolean value
    /// indicating wether the query should continue or not. The method is a specialized version
    /// of `AabbTree::query`, where `search` is defined as `|other_aabb| other_aabb.overlaps(aabb)`
    pub fn query_aabb<F: FnMut(Proxy) -> bool>(&self, aabb: &A, callback: F) {
        self.query(
            |other_aabb| {
                let other_aabb: MinMaxTuple<P> = (*other_aabb).clone().into();
                let aabb: MinMaxTuple<P> = (*aabb).clone().into();
                other_aabb.overlaps(&aabb)
            },
            callback,
        );
    }

    /// Searchs the AABB using a generic `search` function. `search` receives the current
    /// AABB being analyzed and should return true if that AABB is within your search
    /// constraints. Once a leaf node is found and `search` returns true for that node,
    /// `callback` will be called with the node's proxy id as a parameter. The callback
    /// returns a boolean value indicating wether the query should continue or not.
    pub fn query<F, S>(&self, mut search: S, mut callback: F)
    where
        F: FnMut(Proxy) -> bool,
        S: FnMut(&A) -> bool,
    {

        let mut stack = Vec::new();

        if let Some(root) = self.root {
            stack.push(root);
        }

        while let Some(node_id) = stack.pop() {
            if search(&self.nodes[node_id.v].aabb.into()) {
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

    /*
        
    ColliderPairList &AABBTree::ComputePairs(void)
    {
    m_pairs.clear();
    
    // early out
    if (!m_root || m_root->IsLeaf())
        return m_pairs;
    
    // clear Node::childrenCrossed flags
    ClearChildrenCrossFlagHelper(m_root);
    
    // base recursive call
    ComputePairsHelper(m_root->children[0], 
                        m_root->children[1]);
    
    return m_pairs;
    }            
    */

    pub fn compute_pairs(&self, valid_pair_elements: &[Proxy]) -> Vec<(Proxy, Proxy)> {

        let mut pairs = Vec::new();

        let (child0, child1) = match self.root {
            Some(proxy) => match self.nodes[proxy.v].status {
                NodeStatus::Parent(child0, child1) => (child0, child1),
                _ => return pairs,
            },
            _ => return pairs
        };

        let mut seen = Vec::new();

        self.compute_pairs_helper(&mut pairs, &mut seen, valid_pair_elements, child0, child1);

        pairs
    }

    /*
    void AABBTree::CrossChildren(Node *node)
    {
        if (!node->childrenCrossed)
        {
            ComputePairsHelper(node->children[0], 
                            node->children[1]);
            node->childrenCrossed = true;
        }
    }
    */

    fn cross_children(&self, pairs: &mut Vec<(Proxy, Proxy)>, seen: &mut Vec<Proxy>, valid_pair_elements: &[Proxy], proxy: Proxy) {
        if !seen.contains(&proxy) {
            seen.push(proxy);
            if let NodeStatus::Parent(child0, child1) = self.nodes[proxy.v].status {
                self.compute_pairs_helper(pairs, seen, valid_pair_elements, child0, child1);
            }
            else {
                // TODO: encode LeafProxy and ParentProxy into the type system?
                unreachable!("cross_children can't be called with leaf proxies")
            }
        }
    }

    /// TODO trocar self.nodes[proxy0.v] por uma função get_node
    #[inline]
    fn get_node(&self, proxy: Proxy) -> &Node<T, P> {
        assert!(proxy.v < self.nodes.len(), "get_node called with invalid proxy");
        &self.nodes[proxy.v]
    }

    fn compute_pairs_helper(&self, pairs: &mut Vec<(Proxy, Proxy)>, seen: &mut Vec<Proxy>, valid_pair_elements: &[Proxy], proxy0: Proxy, proxy1: Proxy) {

        let node0 = self.get_node(proxy0);
        let node1 = self.get_node(proxy1);

        match (&node0.status, &node1.status) {
            (&NodeStatus::Leaf(_), &NodeStatus::Leaf(_)) => {
                if valid_pair_elements.contains(&proxy0) && valid_pair_elements.contains(&proxy1) &&  node0.aabb.overlaps(&node1.aabb) {
                    pairs.push( (proxy0, proxy1) )
                }
            }
            /*/ 1 branch / 1 leaf, 2 cross checks
            else
            {
                CrossChildren(n1);
                ComputePairsHelper(n0, n1->children[0]);
                ComputePairsHelper(n0, n1->children[1]);
            }*/
            (&NodeStatus::Leaf(_), &NodeStatus::Parent(child0, child1)) => {
                self.cross_children(pairs, seen, valid_pair_elements, proxy1);
                self.compute_pairs_helper(pairs, seen, valid_pair_elements, proxy0, child0);
                self.compute_pairs_helper(pairs, seen, valid_pair_elements, proxy0, child1);
            }
            /*
            // 1 branch / 1 leaf, 2 cross checks
            if (n1->IsLeaf())
            {
                CrossChildren(n0);
                ComputePairsHelper(n0->children[0], n1);
                ComputePairsHelper(n0->children[1], n1);
            }
            */
            (&NodeStatus::Parent(child0, child1), &NodeStatus::Leaf(_)) => {
                self.cross_children(pairs, seen, valid_pair_elements, proxy0);
                self.compute_pairs_helper(pairs, seen, valid_pair_elements, child0, proxy1);
                self.compute_pairs_helper(pairs, seen, valid_pair_elements, child1, proxy1);
            }
            (&NodeStatus::Parent(p0_child0, p0_child1), &NodeStatus::Parent(p1_child0, p1_child1)) => {
                self.cross_children(pairs, seen, valid_pair_elements, proxy0);
                self.cross_children(pairs, seen, valid_pair_elements, proxy1);
                self.compute_pairs_helper(pairs, seen, valid_pair_elements, p0_child0, p1_child0);
                self.compute_pairs_helper(pairs, seen, valid_pair_elements, p0_child0, p1_child1);
                self.compute_pairs_helper(pairs, seen, valid_pair_elements, p0_child1, p1_child0);
                self.compute_pairs_helper(pairs, seen, valid_pair_elements, p0_child1, p1_child1);
            }
        }

    }

    /*
    void AABBTree::ComputePairsHelper(Node *n0, Node *n1)
    {
        if (n0->IsLeaf())
        {
            // 2 leaves, check proxies instead of fat AABBs
            if (n1->IsLeaf())
            {
                if (n0->data->Collides(*n1->data))
                    m_pairs.push_back(AllocatePair(n0->data->Collider(), 
                                            n1->data->Collider()));
            }
            // 1 branch / 1 leaf, 2 cross checks
            else
            {
                CrossChildren(n1);
                ComputePairsHelper(n0, n1->children[0]);
                ComputePairsHelper(n0, n1->children[1]);
            }
        }
        else
        {
            // 1 branch / 1 leaf, 2 cross checks
            if (n1->IsLeaf())
            {
                CrossChildren(n0);
                ComputePairsHelper(n0->children[0], n1);
                ComputePairsHelper(n0->children[1], n1);
            }
            // 2 branches, 4 cross checks
            else
            {
                CrossChildren(n0);
                CrossChildren(n1);
                ComputePairsHelper(n0->children[0], n1->children[0]);
                ComputePairsHelper(n0->children[0], n1->children[1]);
                ComputePairsHelper(n0->children[1], n1->children[0]);
                ComputePairsHelper(n0->children[1], n1->children[1]);
            }
        } // end of if (n0->IsLeaf())
    }
    */
}

/*pub struct CollisionPairIter<T, P, A> 
    where
    P: num::Float + num::FromPrimitive,
    A: Into<MinMaxTuple<P>> + From<MinMaxTuple<P>> + Clone, {
    tree: AabbTree<T, P, A>
}

impl<T, P, A> Iterator for CollisionPairIter<T, P, A> 
    where
    P: num::Float + num::FromPrimitive,
    A: Into<MinMaxTuple<P>> + From<MinMaxTuple<P>> + Clone, {
    type Item = (T, T);
    fn next(&mut self) -> Option<Self::Item> {
        unimplemented!()
    }
}*/

#[cfg(feature = "test_tools")]
#[cfg(test)]
mod tests {
	#![allow(warnings)]
    use super::{AabbTree, NodeStatus, NodeLink, Proxy};
    use super::aabb::Aabb;
    use quickcheck;
    use rand::thread_rng;
    use rand::{SeedableRng, StdRng};
    use std::collections::HashSet;
    use std::hash::Hash;
    use std::fmt::Debug;

    type Vec3 = (f32, f32, f32);
    type A = (Vec3, Vec3);

    /// Returns either a random `AabbTree` or a `quickcheck::TestResult` if the tree is empty.
    fn random_tree(aabbs: Vec<A>) -> Result<(Vec<Proxy>, AabbTree<i32>), quickcheck::TestResult> {
        let mut tree = AabbTree::new();
        let mut proxies = Vec::new();

        for (i, aabb) in aabbs.iter().filter(|aabb| aabb.is_valid()).enumerate() {
            proxies.push(tree.create_proxy(*aabb, i as i32));
        }

        if tree.root.is_none() {
            Err(quickcheck::TestResult::discard())
        } else {
            Ok((proxies, tree))
        }
    }

    fn test_structure<T>(t: &AabbTree<T>, n: Proxy) -> bool {
        let node = &t.nodes[n.v];

        if t.root.unwrap().v == n.v {
            if let NodeLink::Parent(None) = node.link {
            } else {
                return false;
            }
        }

        match node.status {
            NodeStatus::Leaf(_) => node.height == 0,
            NodeStatus::Parent(ref c1, ref c2) => {
                match (&t.nodes[c1.v].link, &t.nodes[c2.v].link) {
                    (&NodeLink::Parent(Some(p1)), &NodeLink::Parent(Some(p2))) => {
                        (p1 == n) && (p2 == n) && test_structure(t, *c1) &&
                            test_structure(t, *c2)
                    }
                    _ => false,
                }
            }
        }
    }

    fn test_metrics<T>(t: &AabbTree<T>, n: Proxy) -> bool {
        let node = &t.nodes[n.v];

        if t.root.unwrap().v == n.v {
            if let NodeLink::Parent(None) = node.link {
            } else {
                return false;
            }
        }

        match node.status {
            NodeStatus::Leaf(_) => node.height == 0,
            NodeStatus::Parent(ref c1, ref c2) => {
                let height1 = t.nodes[c1.v].height;
                let height2 = t.nodes[c2.v].height;

                if (1 + ::std::cmp::max(height1, height2)) != node.height {
                    false
                } else {
                    let c_aabb = AabbTree::<i32>::combined_aabb(
                        &t.nodes[c1.v].aabb,
                        &t.nodes[c2.v].aabb,
                    );

                    c_aabb.min() == node.aabb.min() && c_aabb.max() == node.aabb.max() &&
                        test_metrics(t, *c1) &&
                        test_metrics(t, *c2)
                }
            }
        }
    }

    fn naive_compute_pairs<T>(proxies: &[Proxy], tree: &AabbTree<T>) -> Vec<(Proxy, Proxy)> {
        let mut ret = Vec::new();

        for i in 0..proxies.len() {
            for j in i+1..proxies.len() {
                if tree.get_node(proxies[i]).aabb.overlaps(&tree.get_node(proxies[j]).aabb) {
                    ret.push( (proxies[i], proxies[j]) );
                }
            }
        }

        ret
    }

    fn proxy_list_eq(a: &[(Proxy, Proxy)], b: &[(Proxy, Proxy)]) -> bool 
    {
        let a: HashSet<(Proxy, Proxy)> = a.iter().map(|&(a, b)| if a.v > b.v { (b, a) } else { (a, b) }).collect();
        let b: HashSet<(Proxy, Proxy)> = b.iter().map(|&(a, b)| if a.v > b.v { (b, a) } else { (a, b) }).collect();

        //println!("la {:?} {:?} {}", a, b, a == b);

        a == b
    }

    #[test]
    fn compute_pairs() {
        fn test(aabbs: Vec<(A, A)>) -> quickcheck::TestResult {
            let (proxies, mut tree) = match random_tree(aabbs.iter().map(|&(a, _)| a.clone()).collect()) {
                Ok(tree) => tree,
                Err(res) => return res,
            };
            
            let a = tree.compute_pairs(&(0..1000).map(|i| Proxy{ v: i }).collect::<Vec<_>>());
            let b = naive_compute_pairs(&proxies, &tree);

            let t1 = a.len() == b.len();
            let t2 = proxy_list_eq(&a, &b);

            quickcheck::TestResult::from_bool(t1 && t2)
        }

        quickcheck::quickcheck(test as fn(Vec<(A, A)>) -> quickcheck::TestResult);
    }

    #[test]
    fn movement() {
        fn test(aabbs: Vec<(A, A)>) -> quickcheck::TestResult {
            let (proxies, mut tree) = match random_tree(aabbs.iter().map(|&(a, _)| a.clone()).collect()) {
                Ok(tree) => tree,
                Err(res) => return res,
            };

            for (p, aabb) in proxies.iter().zip(aabbs.iter().map(|&(_, b)| b.clone())).filter(|&(_, aabb)| aabb.is_valid()) {
                tree.set_aabb(*p, &aabb);
            }

            quickcheck::TestResult::from_bool(
                test_structure(&tree, tree.root.expect("root")) &&
                test_metrics(&tree, tree.root.expect("root")),
            )
        
        }

        quickcheck::quickcheck(test as fn(Vec<(A, A)>) -> quickcheck::TestResult);
    }

    /// This makes sure some constraints are respected in random trees. The constraints are:
    /// * Structural validation
    /// Validates every node, checking if the status and links are correct.
    /// * Metrics validation
    /// Makes sure the combined AABB of two children is equal to the parent's AABB, and that the height
    /// is correct.
    #[test]
    fn validation() {
        fn tree_validation(aabbs: Vec<A>) -> quickcheck::TestResult {
            let (_, tree) = match random_tree(aabbs) {
                Ok(tree) => tree,
                Err(res) => return res,
            };

            quickcheck::TestResult::from_bool(
                test_structure(&tree, tree.root.expect("root")) &&
                test_metrics(&tree, tree.root.expect("root")),
            )
        }

        quickcheck::quickcheck(tree_validation as fn(Vec<A>) -> quickcheck::TestResult);
    }

    /// Creates a random tree and then remove the proxies randomly
    #[test]
    fn random_destruction() {
        use rand::Rng;

        fn random_destruction_test(aabbs: Vec<A>) -> quickcheck::TestResult {
            let (mut all, mut tree) = match random_tree(aabbs) {
                Ok(tree) => tree,
                Err(res) => return res,
            };

            /*let mut all = Vec::new();

            tree.query_aabb( &((::std::f32::MIN, ::std::f32::MIN, ::std::f32::MIN), (::std::f32::MAX, ::std::f32::MAX, ::std::f32::MAX)), 
            |v| {
                all.push(v);
                true
            });*/

            thread_rng().shuffle(&mut all);

            for i in 0..all.len()/2 {
                tree.destroy_proxy(all[i]);
            }

            quickcheck::TestResult::from_bool(
                test_structure(&tree, tree.root.expect("root")) &&
                test_metrics(&tree, tree.root.expect("root")),
            )
        }

        quickcheck::quickcheck(random_destruction_test as fn(Vec<A>) -> quickcheck::TestResult);
    }

    /// Checks if every proxy has a status of `Leaf`
    #[test]
    fn inserted_is_leaf2() {
        fn tree_insertion(n: i32) -> bool {
            let mut tree: AabbTree<i32> = AabbTree::new();

            let mut proxies = Vec::new();
            for i in 0..n {
                proxies.push(tree.create_proxy(((-1.0, -1.0, -1.0), (1.0, 1.0, 1.0)), i));
            }

            for (i, v) in proxies.iter().enumerate() {
                if !tree.nodes[v.v].is_leaf() {
                    return false;
                }
                match tree.nodes[v.v].status {
                    NodeStatus::Leaf(Some(t)) => {
                        if i as i32 != t {
                            return false;
                        }
                    }
                    _ => unreachable!(),
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
        fn tree_insertion(aabbs: Vec<A>) -> quickcheck::TestResult {
            let (_, tree) = match random_tree(aabbs) {
                Ok(tree) => tree,
                Err(res) => return res,
            };

            fn test_aabb_sizes<T: ::std::fmt::Debug>(
                t: &AabbTree<T>,
                n: Proxy,
                mut parent_aabb: Option<A>,
            ) -> bool {
                parent_aabb = match parent_aabb {
                    None => Some(t.nodes[n.v].aabb),
                    Some(aabb) => {
                        if aabb.perimeter() < t.nodes[n.v].aabb.perimeter() {
                            return false;
                        }
                        if !aabb.overlaps(&t.nodes[n.v].aabb) {
                            return false;
                        }
                        if !aabb.contains(&t.nodes[n.v].aabb) {
                            return false;
                        }

                        Some(aabb)
                    }
                };

                if let NodeStatus::Parent(c1, c2) = t.nodes[n.v].status {
                    test_aabb_sizes(t, c1, parent_aabb) && test_aabb_sizes(t, c2, parent_aabb)
                } else {
                    true
                }
            }

            quickcheck::TestResult::from_bool(
                test_aabb_sizes(&tree, tree.root.expect("root"), None),
            )
        }

        quickcheck::quickcheck(tree_insertion as fn(Vec<A>) -> quickcheck::TestResult);
    }
}
