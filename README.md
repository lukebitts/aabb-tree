[![Build Status](https://travis-ci.org/lukebitts/aabb-tree.svg?branch=master)](https://travis-ci.org/lukebitts/aabb-tree)

# aabb-tree

[Documentation][docs]

[docs]: http://lukebitts.github.io/docs/aabb-tree/aabb_tree/

This crate is the implementation of a dynamic bounding volume tree based on
axis aligned bounding boxes. It is a spatial structure with support for
querying objects based on their position and size inside the tree.
This work is based on Presson's [`btDbvt`][btdbvt] written for Bullet Physics and
Catto's [`b2DynamicTree`][b2d] written for Box2D.

[btdbvt]: http://bulletphysics.org/Bullet/BulletFull/btDbvt_8cpp.html
[b2d]: https://github.com/behdad/box2d/blob/master/Box2D/Box2D/Collision/b2DynamicTree.cpp

## Example

```rust
extern crate aabb_tree;

use aabb_tree::{AabbTree};

fn main() {
	// Creates a new `AabbTree` where the elements are `i32`s.
	let mut tree : AabbTree<i32> = AabbTree::new();

	// `AabbTree::create_proxy` creates a proxy for an element. Here we are adding the number 0 to the
	// tree in the position (0, 0, 0) with a size of 2³.
	tree.create_proxy(((-1.0, -1.0, -1.0), (1.0, 1.0, 1.0)), 0);

	// `AabbTree::query_aabb` finds every node inside the AABB ((-10.0, -10.0, -10.0), (10.0, 10.0, 10.0))
	// and calls a callback for each result. The callback function returns
	// a `boolean` indicating whether the search should continue or not.
	tree.query_aabb(&((-10.0, -10.0, -10.0), (10.0, 10.0, 10.0)), |proxy|{
		assert_eq!(tree.user_data(proxy), Some(&0));
		true
	});
}
```
