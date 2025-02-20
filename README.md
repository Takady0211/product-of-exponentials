# product-of-exponentials

This repository will be renamed as "PoETLie: Product of Exponentials Toolbox using Lie algebra."

This repository is ROS 2 package for using product of exponentials.

## Why PoE?

The biggest advantage of using product of exponentials method for robotics is its mathematical *simplicity*.
PoE is based on Lie theory, and it enables simple expression and versatile usage for robotics application.
This repository aims not only to utilize PoE for controlling robots but also to introduce PoE itself.

## Mathematical Background

A homogeneous transformation matrix $T$ can be represented by its exponential coordinates $\mathcal{S}\theta \in \mathbb{R}^6$.
Here, $\mathcal{S}$ is the screw axis and $\theta$ is the rotation angle of screw.

```math
\begin{split}
  [\mathcal{S}]\theta \in \mathfrak{se}(3) &\xrightarrow{\mathrm{exp}} T \in \mathfrak{SE}(3) \\
  T \in \mathfrak{SE}(3) &\xrightarrow{\mathrm{log}} [\mathcal{S}]\theta \in \mathfrak{se}(3)
\end{split}
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
