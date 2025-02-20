# product-of-exponentials

This repository will be renamed as "PoETLie: Product of Exponentials Toolbox using Lie algebra."

This repository is ROS 2 package for using product of exponentials.

## Mathematical Background

```math
B \rightarrow \mathbb{R}^6  \\
P \mapsto (\bm{v}, \bm{\omega})
```

## Tips for Debug

Below command shows list of interfaces including custom messages.

```bash
ros2 interface list
```

You don't have to search message type by checking source code, for example:

```bash
$ ros2 interface show geometry_msgs/msg/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
```
