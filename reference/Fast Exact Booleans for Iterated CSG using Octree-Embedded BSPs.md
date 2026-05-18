# Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs

Julius Nehring-Wirxel $^{*}$ , Philip Trettner, Leif Kobbelt 

RWTH Aachen, Lehrstuhl für Informatik 8, Ahornstraße 55, 52074 Aachen, Germany 

![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/bded221bac09c62904845f51c4db166a6575be2aca29f7513d8d31a7b2d97602.jpg)


# ARTICLE INFO

Article history: 

Received 23 July 2020 

Received in revised form 4 November 2020 

Accepted 10 February 2021 

Keywords: 

Plane-based geometry 

CSG 

Mesh Booleans 

BSP 

Octree 

Integer arithmetic 

# ABSTRACT

We present octree-embedded BSPs, a volumetric mesh data structure suited for performing a sequence of Boolean operations (iterated CSG) efficiently. At its core, our data structure leverages a plane-based geometry representation and integer arithmetics to guarantee unconditionally robust operations. These typically present considerable performance challenges which we overcome by using custom-tailored fixed-precision operations and an efficient algorithm for cutting a convex mesh against a plane. Consequently, BSP Booleans and mesh extraction are formulated in terms of mesh cutting. The octree is used as a global acceleration structure to keep modifications local and bound the BSP complexity. With our optimizations, we can perform up to 2.5 million mesh-plane cuts per second on a single core, which creates roughly 40-50 million output BSP nodes for CSG. We demonstrate our system in two iterated CSG settings: sweep volumes and a milling simulation. 

© 2021 Elsevier Ltd. All rights reserved. 

# 1. Introduction

Mesh Booleans or Constructive Solid Geometry (CSG) are popular and intuitive ways to model and design objects. Iterated CSG is the task of applying a sequence of CSG operations on some initial object. For typical use-cases such as carving or sculpting each operation is simple but the resulting object turns more complex the more operations are applied. This setting is also common for simulations of manufacturing processes such as CNC milling or 3D printing. 

Iterated CSG is typically quite challenging since the result of each iteration is used as input for the next operation. Therefore the algorithm is required to be unconditionally robust. Otherwise the simulation degenerates quickly. When the intermediate result grows more complex and is subject to many local modifications, it is unacceptable to use methods where the computation time scales with the complexity of the workpiece. Instead, the runtime of each operation should only depend on the complexity of the actually modified region. These two problems are especially severe in the aforementioned manufacturing simulations as many small, potentially aligned or almost aligned operations are performed. 

While there is extensive literature on how to quickly perform exact Booleans on large input meshes [1-6], the work on iterated CSG is mostly limited to inexact voxel or dexel based methods with their own set of scaling and discretization issues $[7,8]$ . In fact, we found that all modern exact approaches are not well suited for iterated CSG tasks (cf. Section 5). To overcome the stability issues, many exact approaches use arbitrary precision computations $[3,4,9]$ . However, these methods produce meshes with rounded vertex positions after an operation has finished. Extending the lifetime of the arbitrary precision numbers across many iterations is usually not feasible as the quantization complexity increases rapidly after each iteration. Consequently, current solutions suffer from either stability issues or poor performance. 

Our proposed solution is based on Binary Space Partition (BSP) trees. BSPs have already been used to perform robust CSG operations with a plane-based geometry representation $[1,2]$ . However, they require conversion from mesh to BSP (import) and BSP to mesh (extraction), both with their own robustness issues. While fast for small trees, BSP-based Booleans, in the worst-case, scale at least with $\mathcal{O}(n^{2})$ for n nodes, even if the output complexity is only $\mathcal{O}(n)$ $[10]$ . Extracting a polygonal mesh from a BSP is conceptually simple: start with a cube large enough to contain the result and recursively perform mesh-plane cuts until the leaves of the BSP are reached. This results in a set of convex meshes from which those belonging to out nodes are discarded. With infinite precision, each intermediate mesh is perfectly convex. However, rounding caused by fixed-precision floating point arithmetics often leads to small non-convexities. This greatly complicates further cutting steps with epsilon tolerances and topology-repair steps $[11]$ . These problems are basically guaranteed to happen if the BSP is constructed from a triangle mesh: normally, three planes intersect in a single point and it is unlikely that a fourth plane contains the same point. In a triangle mesh, the average number of faces around a vertex is six, meaning that, by construction, six planes should intersect in a single point, something extraordinarily unlikely when working with floating point values. A stable extraction can still be achieved by either accepting these inaccuracies and formulate a topologically robust cutting [11] or by using plane-based geometry and exact predicates [1,2]. We use fixed-precision integer arithmetic (Section 3.2) to provide a fast and reliable numerical foundation which enables us to formulate a simple, yet exact and efficient algorithm for extracting the mesh surface of a BSP tree (Sections 3.3 and 3.4). 

Even if the numerical substrate is fast, BSP-based CSG still suffers from scaling problems. Therefore, we superimpose a global octree structure and store a BSP in each octree cell. These octree-embedded BSPs should not be seen as a temporary constructs such as in $[2]$ but instead as a volumetric data structure that persists through the entire sequence of CSG operations. While the octree introduces some overhead as the cell borders split triangles, it imposes strict limits on the BSP complexity and thus mitigates the scaling issues. Additionally, it provides localization of operations and a set of efficient culling operations to limit the CSG computation to the affected regions. An octree can be seen as a special BSP tree, a fact that we exploit to formulate efficient merge and split operations. Still, the special structure of an octree guarantees good worst-case behavior that cannot be provided by more general BSPs. Conceptually, our octree-embedded BSPs can be seen as a single gigantic BSP where the upper levels follow an octree structure to provide spatial partitioning and only the lower levels adapt to the geometry. 

Finally, we added various improvements to the classical mesh import, BSP Booleans, and mesh extraction algorithms. Importing a mesh into our data structure is fast with our exact computations as we can first build the octree and then locally construct BSPs using a small number of triangles for each octree cell. With floating point computation this tends to produce cracks at octree cell borders. 

# 1.1. Contribution

In summary, we present 

- octree-embedded BSPs as a data structure for performing high-performance iterated CSG operations. 

- exact and efficient CSG based on BSP Booleans using custom-tailored 256 bit integer arithmetic. 

- a BSP to mesh conversion using an efficient algorithm to cut planes against convex half-edge meshes. 

# 2. Related work

There is extensive literature for Booleans on meshes $[1,2,9,12–15]$ . The trilemma for different approaches are essentially performance, accuracy, and algorithmic stability. Usually, only two of these can be fulfilled in a satisfying manner. The main causes for these problems are rounding issues with fixed size floating point numbers, poor performance of exact calculation schemes, and high algorithmic complexity due to an immense number of topological configurations. 

Binary Space Partition based approaches were kick-started by Naylor et al. [12]. They introduced a procedure to merge two BSPs into a single one that represents a Boolean of the two inputs. To prune empty nodes from the resulting BSP, a polygon cutting algorithm is part of the merge procedure. Additionally, the conversion from and to a mesh requires the same polygon cutting algorithm. Unfortunately, polygon cutting tends to be highly unstable when using fixed length floating point arithmetics [10]. Lysenko et al. [10] present an improved BSP CSG algorithm by eliminating the polygon cutting from the merge procedure and replacing it by a satisfiability check for a linear program. The constraints of the program are the same as the half-spaces from the current subtree to the root of the BSP. If there is no solution to the linear program, the subtree is empty. 

Bernstein and Fussell [1] go a different route to deal with instability. They propose using exact filtered predicates [16] in the domain of BSP merging. To avoid having to increase the resolution when creating new intersection vertices they use a purely plane-based representation for convex polygons which is a subset of the class of Nef polygons [17]. Plane-based modeling itself was already proposed by [18]. Vertices are defined as the intersection of three non-coplanar planes, convex polygons consist of a supporting plane and a list of planes determining the boundary of the polygon. Unfortunately the BSP merging procedure scales poorly with the size of the input BSP [10]. To mitigate the scaling problems Campen and Kobbelt [2] embed Bernstein's BSP merging algorithm into an octree that localizes CSG operations. Only small, overlapping parts of the input meshes are converted to BSPs, limiting the complexity of each single BSP merge. The result is converted back to the original mesh. They also give guarantees for exactness of the operations as well as the conversion from and to mesh representation. However, these guarantees force them to limit the precision of the input mesh. 

Since the conversion between BSP and mesh is not trivial, Wang and Manocha $[11]$ propose an efficient algorithm to extract a mesh from a BSP. They do not use exact calculations but instead achieve stability by making sure that their mesh data structure is always in a sound state topologically. 

To avoid conversions to and from plane-based representations, many approaches perform CSG operations directly on the mesh. Some avoid floating point issues by approximating a seam where the two input meshes intersect $[14,19,20]$ . Others present methods that perform well on many meshes but rely on careful tuning of epsilon values and may still fail $[21,22]$ . Barki et al. $[9]$ first compute the intersections of all triangles of the input meshes using exact rational number representations. They then determine which triangles belong to the result of the Boolean using an ordering of triangles around their shared edges. Sheng et al. $[4]$ pursue a similar approach: they first cut intersecting input triangles, but instead of using exact rationals to represent the new primitives, they use plane based vertex, edge, and face representations similar to $[1]$ . Magalhães et al. $[6]$ perform exact calculations on volume meshes and additionally use symbolic perturbation to avoid many special cases. Using actual vertex perturbation, Douze et al. $[5]$ introduce a very fast but unstable algorithm that executes successfully on a large number of meshes. 

Besides the purely mesh or plane-based methods, Hachenberger et al. [13] introduced a Nef-polyhedral-based approach that is implemented in the CGAL library. It is both exact and has good stability, but is quite slow and its internal data-structures are very memory consuming. 

In addition, there are a few alternative approaches: Zhou et al. [3] combine exact calculations with a winding number based approach. They derive the winding number of vertices from one mesh in respect to another input mesh topologically and use that information to determine which parts of the mesh to keep. Notably, they provide a good implementation of their method in libigl [23]. Recently, Cherchi et al. [24] improved the performance of this approach by deriving lazy floating-point predicates to replace the exact rational arithmetic used by [3]. Similarly, Barill et al. [25] introduce a general approach to determine winding numbers as a measurement of insideness which can be used to perform Boolean operations. 

Beside the mesh or plane based approaches, voxel-based approaches are also often used for CSG operations [7,26-28]. By principle they suffer from aliasing effects and limited resolution. Additionally, the machining industry often uses a dexel-based representation $[8,29]$ . Dexel, also called multi-layer-z-map are essentially a 2-dimensional regular grid with a list of height values that describe a transition from inside to outside or vice versa. While this representation is often extended into three dimensions, it still suffers from aliasing problems similar to voxel-based approaches. 

Our proposed method builds upon Bernstein and Fussell [1] and Campen and Kobbelt [2]. Instead of using filtered floating-point numbers, we show how the required geometric predicates can be formulated and implemented using fixed-width integers. By limiting the largest intermediate value to 128, 192, or 256 bit, we present a set of extremely practical speed-precision tradeoffs. Furthermore, we show that instead of storing vertices as the intersection of three planes, a representation as 4D homogeneous coordinates has several advantages. In particular, the most expensive predicate does not involve a $4 \times 4$ determinant anymore but rather a cheaper 4D dot product. Algorithmically, we present a fast plane-against-convex-mesh cutting algorithm that allows us to implement exact BSP booleans efficiently using mesh extraction. To the best of our knowledge, this results in the fastest exact BSP booleans to date and simultaneously provides high quality surface meshes and a method for reducing BSP redundancies. Finally, our octree and the one from Campen and Kobbelt [2] serve subtly different purposes: Theirs is a temporary acceleration structure used to find and localize the intersecting surface. Our octree-embedded BSPs are a persistent data structure that are kept across iterations and could be considered an “imposed structure” on the upper levels of a global BSP. 

# 3. Method

# 3.1. Overview

Our method builds on the ideas of [1,2,12]. Geometry is represented as a BSP tree where each leaf node has a label (in and out for simplicity but the algorithms are trivially extended to a multi-material setting). Each non-leaf BSP-node contains a plane that splits the domain into a positive and a negative half-space. Thus, BSP trees represent a hierarchy of convex polyhedral cells and the object geometry is the union of all in-labeled leaf cells. Instead of using floating point numbers, we base our entire system on integers and guarantee exactness of all results by analyzing and bounding the required number of bits (Section 3.2). By using fixed-precision instead of arbitrary-precision arithmetic we can cater to all practical scenarios with minimal performance overhead. Our system uses a plane-based geometry representation where each plane equation has integer coefficients. While all input points must be integer, the resulting vertices are represented by the intersection of three planes (as 4D homogeneous coordinates) and can thus lie at fractional positions. Because CSG operations cannot introduce new planes, all possible positions are representable using combinations of input planes. 

Equipped with fast and exact predicates, we present a practical and efficient algorithm for converting BSP trees into meshes (Section 3.4). Instead of using convex Nef polyhedra [1,17], our boundary extraction algorithm operates on convex polyhedra stored in a half-edge data structure with a fast plane-mesh cutting procedure as its core operation. This algorithm significantly outperforms the state of the art such as [3,5,13,30] while staying completely exact. 

Boundary extraction is typically a slow, error-prone, and often overlooked step. However, based on our fast and exact extraction scheme, other essential BSP operations can be implemented. Section 3.5 describes how performing CSG on BSPs is straightforward and faster than the state of the art when using our boundary extraction as a building block. Removing redundancy from BSPs can be implemented by extracting the represented surface and converting it back to a BSP without any loss (Section 3.7). A typical workflow of our method is depicted in Fig. 1. 

![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/1b9f301730a86d5beccf269e05bcb855002715ab11504508decdfe5d62a82a6b.jpg)



Fig. 1. Performing Booleans on an octree-embedded BSP data structure and another BSP can lead to suboptimal resulting BSPs. Extracting and reimporting the BSP's boundary usually improves the BSP quality.


Section 3.8 presents our octree-embedded BSPs, a global octree that stores a BSP in each cell and locally adapts to the BSP complexity. When performing iterated CSG, we maintain the octree across iterations and perform octree-BSP merges in each step. 

The final module of the system is a pair of algorithms to import triangular meshes into our data structure: from polygon soup to BSP and from triangle soup to octree-embedded BSP. 

# 3.2. Arithmetic foundation

Floating point computation often introduces round-off errors that tend to violate geometric invariants and threaten algorithmic stability. Using arbitrary-precision libraries such as gmp $[31]$ typically incur prohibitive performance overhead. Instead, we base our computation on fixed-precision integer arithmetic and exact representations. We start by introducing our required operations. Then, given a maximum number of bits that we are willing to use for intermediate results, we work backwards to determine the largest range of input values that guarantees overflow-free computation. 

Input meshes are given with integer vertex positions and (non-normalized) integer normals. Inner BSP nodes store planes p in plane equation form with integer coefficients $a_{p}, b_{p}, c_{p}, d_{p}$ where $(a_{p}, b_{p}, c_{p}) = n_{p}$ is the plane normal and $d_{p} = -v^{T}n_{p}$ is the offset from the origin (for any point v that lies on the plane). While input vertices are integer, the result of CSG operations can contain vertices at fractional positions. Thus, in a plane-based geometry representation, we do not actually compute vertex positions but treat each vertex as the intersection of three planes. Bernstein and Fussell [1] describe working with planes and intersection of planes in the context of BSP booleans: Given two planes p and q, 

$$
n _ {p} \times n _ {q} = \vec {0} \tag {1}
$$

tests if two planes are coplanar. Three planes $p, q, r$ intersect in a unique point if the determinant 

$$
\left| n _ {p} n _ {q} n _ {r} \right| \neq 0. \tag {2}
$$

BSP extraction and mesh cutting require another operation: given a point (as the intersection of planes $p, q, r$ ) and a plane $s$ , classify if the point lies on the positive side, on the negative side, or exactly on $s$ . This can be computed using the sign of a $4 \times 4$ times a $3 \times 3$ determinant: 

$$
\operatorname{sign} | p q r s | \cdot \operatorname{sign} \left| n _ {p} n _ {q} n _ {r} \right|. \tag {3}
$$

Note that these tests can be performed with integer arithmetic exclusively. 

In this formulation, vertex classification requires computing an expensive $4 \times 4$ determinant. We derive a different but equivalent way to classify vertices based on homogeneous coordinates. While our version basically results in the same formula, we offer an interpretation that immediately clarifies why a large part of the computation does not depend on s and can therefore be precomputed and reused. Given three planes p, q, and r, their intersection point can be found by solving 

$$
\left( \begin{array}{l} n _ {p} ^ {T} \\ n _ {q} ^ {T} \\ n _ {r} ^ {T} \end{array} \right) x = \left( \begin{array}{c c c} a _ {p} & b _ {p} & c _ {p} \\ a _ {q} & b _ {q} & c _ {q} \\ a _ {r} & b _ {r} & c _ {r} \end{array} \right) x = \left( \begin{array}{l} - d _ {p} \\ - d _ {q} \\ - d _ {r} \end{array} \right). \tag {4}
$$

Using Cramer's rule, $x = (|A_1|, |A_2|, |A_3|) / |A|$ , where $A = (n_p, n_q, n_r)^T$ and $A_i$ is $A$ with the $i$ th column replaced by $(d_p, d_q, d_r)$ . Written in 4D homogeneous coordinates, $x = (|A_1|, |A_2|, |A_3|, |A|)$ , or equivalently 

$$
\operatorname{intersect} (p, q, r) = \left( \begin{array}{c} x _ {1} \\ x _ {2} \\ x _ {3} \\ x _ {4} \end{array} \right) = \left| \begin{array}{c c c c} \mathbf {e} _ {1} & \mathbf {e} _ {2} & \mathbf {e} _ {3} & \mathbf {e} _ {4} \\ a _ {p} & b _ {p} & c _ {p} & d _ {p} \\ a _ {q} & b _ {q} & c _ {q} & d _ {q} \\ a _ {r} & b _ {r} & c _ {r} & d _ {r} \end{array} \right|, \tag {5}
$$

where $e_{i}$ are the canonical unit vectors. We classify points x relative to plane s by computing the sign of the distance to the plane $n_{s}^{T}x + d_{s}$ . When given in homogeneous coordinates, we can avoid fractional results by multiplying with $x_{4}$ and accounting for its sign: 

$$
\operatorname{classify} (x, s) = \operatorname{sign} \left(x ^ {T} s\right) \cdot \operatorname{sign} \left(x _ {4}\right) \tag {6}
$$

In contrast to Bernstein and Fussell [1], we store vertex positions as easily interpretable homogeneous 4D integer coordinates. The costly $4 \times 4$ determinant for classifying vertices is split into the construction of the intersection points (pre-compute four $3 \times 3$ determinants) and the actual classification (a 4D dot product). In our mesh cutting algorithm, classification is performed more frequently than intersection construction, thus resulting in a significant overall performance gain. (See Table 1 in the Evaluation for benchmarks.) 

When exact computation is required, typical approaches use either arbitrary-precision libraries (like gmp) or filtered floating-point predicates (like Shewchuk [16]). We found these either too slow or having insufficient resolution. Instead, we use fixed-precision integers and analyze the operand bounds of each operation to choose appropriate bit widths. Modern x64 CPUs allow highly efficient implementation of wide integer addition and multiplication. Chaining the adc instruction (add with carry) k times implements $64 \cdot k$ bit integer addition. Multiplication is efficient because the mul instruction multiplies two 64 bit integers and stores the 128 bit result in two registers. This can be used as a building block for a simple long multiplication scheme. For very large numbers, this scheme is inefficient as it scales with $\mathcal{O}(n \cdot m)$ (given n and m bit inputs) but we found it well suited for integers consisting of only a few 64 bit blocks. 

Given a budget of up to $b$ bits per operation, we can determine the input precision tradeoffs for normals and vertex positions of our method. Let the biggest absolute coordinate value of any input vertex be $v^{+}$ and similarly that of any input normal be $n^{+}$ . Similarly, $d^{+}$ is the bound for the plane distance coefficient. The largest intermediate result is $x^{T}s$ and must not exceed $\pm 2^{b - 1}$ (ignoring $-2^{b - 1} - 1$ for simplicity). $x^{T}s$ is the same as $|pqr s|$ where $p, q, r$ are the planes that $x$ was created from. Hadamard [32] showed that a $4\times 4$ determinant with matrix entries $\in [-1,1]$ cannot exceed 16. $|pqr s|$ has three columns bounded by $n^{+}$ and one by $d^{+}$ . Thus, linearity of the determinant implies that $|pqr s|$ cannot exceed $16\cdot n^{+3}\cdot d^{+}$ . For our planes, $d = -v^{T}n$ for some input vertex $v$ , so we can conservatively estimate $d^{+}$ as $3 \cdot v^{+} \cdot n^{+}$ . In summary, given $b$ bit integer operations, we can guarantee overflow-free exact predicates if the following is satisfied: 

$$
n ^ {+ 4} \cdot v ^ {+} \leq \frac {1}{4 8} \cdot 2 ^ {b - 1} \tag {7}
$$

Choosing $n^{+}$ and $v^{+}$ independently is beneficial for applications that construct planes directly, but when importing a triangle mesh, normals are computed from vertex positions via $e_{12} \times e_{13}$ where $e_{ij} = v_{j} - v_{i}$ are the triangle edge vectors. A (very) conservative estimate of $e^{+}$ would be $2 \cdot v^{+}$ and thus $n^{+} = 2 \cdot e^{+2} = 8v^{+2}$ due to the cross product. Thus, if normals are computed from vertex positions, the condition simplifies to 

$$
v ^ {+ 9} \leq \frac {1}{8 ^ {4} \cdot 4 8} \cdot 2 ^ {b - 1} \tag {8}
$$

or equivalently 

$$
v ^ {+} \leq 0. 2 3 9 \cdot 2 ^ {b / 9} \approx 2 ^ {b / 9 - 2}. \tag {9}
$$

Therefore, our b = 256 bit arithmetics can exactly import all integer meshes with positions smaller than $8.73 \cdot 10^{7}$ , or slightly above 26 bit. For comparison, single precision floating point has 23 bit mantissa and most ASCII meshes we tested do not exceed 7 significant decimal digits. Fig. 2 shows different tradeoffs for varying bit widths b. 

For a simulation with a bounding volume of $1 \, m^{3}$ , we get a resolution of 0.22 mm for 128 bit, $1.58 \, \mu m$ for 192 bit, and 11.5 nm for 256 bit, assuming we want to guarantee that any input normal can be exactly represented. In Section 4.1 we evaluate the performance of operations described in this section and compare 128 bit, 192 bit, 256 bit, and gmp versions. 

# 3.3. Mesh cutting

The core of our method is the following operation: cut a convex mesh into two parts along a given plane. With our numerical foundation, all results are exact and no epsilon tolerance is required. By modifying the mesh in-place and trying to minimize how many geometric primitives need to be processed we obtain a highly efficient cutting procedure. 

This operation serves as the basis for boundary extraction (Section 3.4), BSP Booleans (Section 3.5), and BSP simplification (Section 3.7). Previous methods use cutting for these as well but suffered from stability issues and overall quadratic scaling [1,10, 12]. 

![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/d6147e27adb6b35cc4a2cca97939f146a87aec086a658cb1b8c0e95927c9e5a3.jpg)



(a) edge descent


![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/a4e5fb4a3b09414b86575b3608ee933d463f36d0baadfad9aa81d37dc2c4f55a.jpg)



(b) marching


![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/d466bf3452143b0176a947e27ab84bf4aa9ee51c55b948008cc77f3ddd555cef.jpg)



(c) result



Fig. 3. (a) In the edge descent step, our mesh cutting algorithm first finds an edge intersecting the cut plane (red edge) by following vertices with shorter distance-to-plane (blue edges). Only vertices along the way are tested (red). (b) In the marching step, we cut each affected face and insert cut edges (green) until reaching the starting point again. Again, only the red vertices need to be classified. (c) The result is the mesh cut in-place into lower and upper part (explosion view).


![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/0206bca4781d0815ef6aa037ca6681c51a20a1bd997e83cfbb92e8a12b5cea26.jpg)



Fig. 4. During the marching there are only two distinct cases when splitting a single face: (a) The face is traversed until an edge with vertices on opposite sides of the cutting plane is found. The edge is then split and we are now in case (b) A vertex lies exactly on the cutting plane. The vertex is duplicated and two new edges on the cutting plane are inserted.


Fig. 3 illustrates our algorithm. Given a starting halfedge and the cutting plane, we traverse from vertex to vertex, always choosing the neighbor vertex with smallest distance to the plane, a process we call edge descent. We stop when reaching a vertex on the opposite side of the plane. If no such vertex can be found, the edge descent terminates in a global minimum (all submeshes are convex) and we know that the plane does not properly intersect the mesh. In our case, a proper intersection means that both resulting meshes from the cut have non-zero volume. For example, a non-proper intersection would be a cut plane that is coplanar with a mesh face. In case of a proper intersection, we always find an edge that intersects with or a vertex that lies on the cutting plane. From there we follow the faces-to-cut around until we reach the initial edge again. Along the way, we add the new faces and edges produced by the cut and locally apply the necessary changes to the mesh topology. We call this step marching. The decision which edge or vertex lies on the cutting plane uses the classification function from the previous section. With exact computations, the meshes stay perfectly convex after cutting. During marching, only two different cases can occur: we find the next edge intersecting the plane or we find a vertex lying on the plane (see Fig. 4). If an edge lies within the cutting plane, no neighboring face needs to be split. 

At the end, the single convex mesh is split into two parts, one belonging to the positive half-space defined by the plane and the other to the negative half-space. Two new planar faces are inserted. In case one of the parts has zero volume, the plane does not have a proper intersection and the mesh is not changed. The main reason why this algorithm is fast is that most of the time only a fraction of the mesh is traversed and the modification is performed in-place. Our experiments in Fig. 9 demonstrate superior results compared with the naive, classify-all-vertices method. Empirically, we observe that the per-cut costs grow sub-linear with the mesh size. 

It is important to note that the resulting mesh has neither integer nor floating-point coordinates. Instead, all vertices are represented as the homogeneous 4D integer coordinates introduced in Section 3.2, especially Eq. (5). This makes the cutting operation exact and fully robust, allowing us to use it as a basis for the boundary mesh extraction and even the BSP booleans. 

There is a certain similarity with the GJK algorithm $[33]$ for collision detection between two convex objects. While our plane-mesh cutting traverses from edge to edge, choosing those that reduce the distance to the cutting plane the most, the GJK algorithm traverses from simplex to simplex, choosing the simplex closest to the origin. On a meta level, both algorithms exploit that, on a convex polyhedron, we can minimize certain distance functions by greedily follow edges or sub-simplices without running into local minima. 

# 3.4. Boundary extraction

Based on mesh-plane cutting, we can formulate an efficient boundary extraction algorithm. The task is, given a BSP tree and a bounding box, to compute a mesh of the represented surface inside the bounding box. Our algorithm works in two steps. 

First, we compute convex meshes for each BSP leaf node labeled in. This is done recursively by starting with a mesh for the bounding box and applying our cutting operation for each inner node of the BSP. During cutting we annotate each face with the index of the opposite BSP node. All out leaves are discarded. 

While these convex meshes can be used for rendering, they contain many interior (double) faces. Thus, in a second step, we remove all parts of the faces that are not in-out transitions. For each face we know the opposite BSP subtree and cut the face recursively against this tree. This procedure is shown in Fig. 6. This is a simple convex face against plane cutting, equivalent to polygon clipping. When the leaf nodes are reached we can classify the face pieces as in-out or in-in, discarding the latter. Fig. 5 shows the result of this step. 

Note that only clipping faces against the opposite BSP will produce T-junctions that might produce problems with rendering and further processing. However, during clipping we can again store the opposite BSP and afterwards subdivide polygon edges according to this opposite BSP, introducing valence-2 vertices. Take for example the upwards pointing face in the middle of Fig. 5(c). After the polygon clipping, this is a rectangle, resulting in a non-conforming mesh with a T-junction. After introducing the valence-2 vertices, it topologically becomes a pentagon. Thus, in our 3D use case, the procedure in Fig. 6 is actually applied two times: First, to clip the boundary polygons to get rid of inner faces. Second, to subdivide boundary edges to prevent one-sided T-junctions. 

![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/93e0b969abdb80707868249c45086b17ac16ec584b6acd6f17f5e0bba3c034ba.jpg)



Fig. 5. Exact mesh boundary extraction. Given an input BSP (a), we use our mesh cutting to extract convex polyhedra for each BSP leaf, discarding out-cells (b, explosion view). The in-cells can contain overlapping interior surfaces which are removed by a 2D face-plane clipping (c, explosion view). The resulting mesh (d) exactly describes the surface represented by the BSP (light blue are octree cell boundaries, darker blue result from BSP planes).


![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/104e13d1b5feb0ff57ea66130097b79c7587ebe52aa50fc47fcbdd434582e94b.jpg)



Fig. 6. Every time a polyhedron is cut during boundary extraction the two new faces inside the cutting plane are assigned the BSP node on the opposite side of the cutting plane (left and center). To extract the exact boundary, faces not pointing to leaf nodes are cut again according to their opposite sub-BSP (right). Boundary faces (orange) belong to BSP nodes marked as in with opposite nodes out.


# 3.5. Booleans on BSPs

Given two BSPs representing geometry, Naylor et al. [12] describe how to merge these. The merge procedure takes two BSPs and a merge function $f_{m}(s_{a}, s_{b}) \to s$ , where $s_{a}, s_{b}, s \in \{in, out\}$ are leaf node labels. The choice of $f_{m}$ determines which Boolean operation is performed. For example, the union is defined by the function that only returns out when both $s_{a}$ and $s_{b}$ are out. 

We formulate the BSP merge based on our efficient mesh cutting. Starting with a cube representing the bounding volume and root nodes A and B of two BSPs, we define the following merge operation that recurses on A: 

1. Terminate if either A or B is a leaf node (based on the merge function, either in, out, A, B, or leaf-inverted versions of A or B are inserted at the current node) 

2. Cut A's plane against the current mesh 

(a) if $A$ 's plane produces a proper intersection with the current mesh, recursively call merge(A.left, B) and merge(A.right, B) 

(b) otherwise recursively call merge $(A', B)$ where $A'$ is the child of $A$ that contains $B$ 

This algorithm produces a BSP that corresponds to the chosen Boolean operation applied to the two input BSPs as well as a polygonal mesh. The mesh consists of a set of convex polyhedra that can each be mapped to one BSP node of the output BSP. Note that they may not always correspond to a leaf node: The merge procedure never recurses B but instead may copy B or its leaf-inverse directly into the output BSP. In that case the corresponding polyhedron belongs to B which may be an inner node. In the resulting BSP some nodes that belong to B may not produce any volume because the planes of B are not guaranteed to intersect with its corresponding polyhedron. In our implementation we circumvent this by also fully cutting the corresponding polyhedron according to B. This fits nicely with our pipeline since each Boolean operation is followed by a Redundancy Removal step (see Section 3.7) where the cut mesh is required anyways. 

# 3.6. BSP import

For converting a mesh into BSP representation we apply a commonly used algorithm $[12]$ that iteratively builds a BSP from the given input faces. At first, the BSP is initialized as a single leaf node. During each iteration, an input face is chosen and inserted into the BSP. Traversal starts at the root node. Traversed leaf nodes are replaced by inner nodes with the plane equal to the supporting plane of the inserted face and traversal stops. The plane's normal direction determines which children of the new node are marked in or out. When an inner node is encountered there are three different cases: Either the face intersects with the node's plane. It is then split along the plane resulting in two faces, each traversing its corresponding subtree. If the face lies completely on one side of the node's plane, it traverses only that subtree. Finally, if the face is coplanar to the node's plane, the face is discarded. This step is important for the merging of split faces in the redundancy removal step (Section 3.7). 

To avoid stability issues caused by floating point inaccuracies, the face-splitting algorithm must be performed in an exact manner. Therefore we use the same plane-based vertex representation as presented in Section 3.2. Even if the input faces consist only of triangles, intermediate faces are generally arbitrary convex polygons with corners defined by plane intersections. Retriangulation is not possible because it requires construction of new planes (red in the inset) from points that are not original input vertices and thus may lie on fractional coordinates. Such new planes might then require more precision than we have available and therefore cannot be constructed in the general case. 

![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/62bfbf5be2811492064e65b9637acc496605180bf612252a21ee1951c429f331.jpg)


A crucial step for creating well balanced BSPs is the choice of the next face that is inserted into the BSP. Heuristics that aim to improve the balanced-ness of the BSP are often expensive and scale poorly $[10]$ , as they often require global knowledge. As our BSPs are naturally limited by their enclosing octree and their number of inner nodes is bounded, we found that choosing the faces in a random order is sufficient in our case. 

![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/02da331bf8af1c6bf39587ce39df53f7c35ef7323d535178a8aaeaf0b6c90e09.jpg)



Fig. 7. Each leaf of the octree contains a BSP. The BSP size is the subdivision criterion for each octree cell. The surface is color-coded by octree cell.


# 3.7. BSP redundancy removal

The result of CSG operations on BSP trees often produces suboptimal BSPs that contain redundant or unnecessary nodes. Fig. 1 shows a simple example. Although we are unaware of methods that produce optimal trees, a common way to improve the BSP structure is to extract the corresponding mesh and rebuild a new BSP from scratch. This quickly becomes infeasible in an inexact setting as rounding errors accumulate and prevent the rebuilt BSP from representing the exact same volume as the original BSP. However, our boundary extraction from Section 3.4 is fast and exact, making the simplification-through-rebuilding approach effective. In practice, we perform the simplification after each BSP merge. This means little additional computation is required to perform the boundary extraction: The merge procedure already produces a convex cell for each leaf node. What remains is the removal of interior overlapping surfaces (cf. Section 3.4). 

Other exact methods, like $[1-3]$ , could apply this approach as well. However, this incurs a significant performance penalty when used without our fast mesh cutting from Section 3.3. 

# 3.8. Octree-embedded BSPs

BSPs offer a well defined method to perform CSG operations. However, these methods scale superlinearly with the size of the BSP (see Fig. 10). To circumvent this, we embed BSPs with a limited number of nodes into a global octree. An example of this is shown in Fig. 7. Changes to the geometry may cause a BSP to exceed the maximal number of nodes. In that case its corresponding octree cell is subdivided into eight equally sized child cells. The smaller child bounding boxes paired with a redundancy removal step cause a decrease in BSP sizes per octree cell, with the exception of a few pathological cases. For example, consider a valence 250 vertex without coplanar adjacent faces that lies at a fractional coordinate. The cell containing this vertex cannot be subdivided to contain less than 250 BSP nodes. In that case, some higher complexity nodes have to be accepted. 

![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/0e4c04861e52fee006444da556cdce10d9acee195d3d81ecfe09fb09a8bb2867.jpg)



Fig. 8. Subdividing an octree cell is equivalent to intersecting the cells BSP with the bounding box of each of its child cells. For combining the children of a cell, their BSPs are inserted into the leaf nodes of a BSP that contains the axis-aligned planes separating the children (COMBINE A). This is followed by a step (COMBINE B) of redundancy removal (cf. Section 3.7) which eliminates the planes from the axis-BSP.


It may be necessary to perform multiple splits until the BSP size has dropped under the given threshold. Since the complexity of the merge procedure can result in quadratic runtime and output size, smaller BSP-trees perform better. However, reducing the maximal BSP size causes an increase in octree refinement. Experimentally, we found a good compromise for the BSP threshold between 100 to 200 (cf. Section 4.5). To assure the octree complexity stays low, octree leaf nodes that have a combined complexity small enough to fit into a single octree cell are merged together. 

As octrees are a special form of BSP trees, cell subdivision and merge can be implemented efficiently using our BSP algorithms. A 2 dimensional version of the process is shown in Fig. 8. Subdividing an octree cell is equivalent to computing the intersection of its BSP with the boundary of each of its children. The opposite – combining eight leaf cells into one – is technically the union of the eight child BSPs. However, this requires the inclusion of their boundaries, as BSPs are in general unbounded. The outer boundary is already implicitly given by the parent octree cell. To represent the inner boundaries that separate the eight children from each other, we build an axis-BSP. The first three of its levels each contain planes that separate the child cells along one axis. The eight leaf nodes are then replaced with the BSP from their corresponding octree cell. This technically concludes the combining step. However, the additional planes introduced by the axis-BSP are rarely part of the solid boundary and the combined child BSPs often share duplicate planes. Therefore the combine step is always followed by a redundancy removal (cf. Section 3.7). 

# 3.9. Octree import

We have already described how to convert a mesh into a BSP. After the input mesh has been rounded once to integer coordinates (according to Section 3.2) further operations make use of exact calculations. 

Converting a mesh into the octree could be achieved by creating a single BSP in the octree root and then subdividing until the threshold for each octree cell is reached. However, the subdivision scales poorly with the large BSP size. Instead, we first create a triangle octree. The splitting criterion of an octree cell is first based on the number of triangles. The result is an octree with triangles in some leaf cells. Next, triangles are clipped to their octree node boundary and are then used to create a BSP for the given cell. The clipping is necessary to ensure the correct creation of BSPs. After creating the BSP, the octree cell complexity bound is enforced by another round of subdivisions, which is now based on the actual BSP sizes. At this point, all octree cells that contain parts of the input geometry store a valid BSP. To correctly classify the remaining octree cells we perform a flood-fill (on octree cells) to propagate the correct in and out labels. 


Table 1
Performance of the elementary operations used in our method in CPU cycles. We compare our custom 128 bit, 192 bit, and 256 bit arithmetic against arbitrary-precision arithmetic provided by gmp [31] and a lazy exact implementation from CGAL [34], Lazy_exact_nt<Quotient<MP_Float>>. gmp and CGAL were tested with the same numbers as the 256bit arithmetic. Due to their internal allocations, the timings of gmp and CGAL showed non-negligible variance of about 5%-10%.


<table><tr><td>Operation</td><td>128b</td><td>192b</td><td>256b</td><td>CGAL</td><td>gmp</td></tr><tr><td>plane_from_points</td><td>226</td><td>512</td><td>761</td><td>2770</td><td>9100</td></tr><tr><td>are_planes_parallel</td><td>5</td><td>15</td><td>15</td><td>910</td><td>1840</td></tr><tr><td>signed_distance</td><td>4</td><td>5</td><td>11</td><td>530</td><td>1360</td></tr><tr><td>to_double_position</td><td>5</td><td>62</td><td>88</td><td>31</td><td>120</td></tr><tr><td>intersect_3_planes</td><td>23</td><td>160</td><td>402</td><td>4200</td><td>6380</td></tr><tr><td>classify_vertex</td><td>13</td><td>103</td><td>142</td><td>710</td><td>1540</td></tr></table>

Note that the triangle clipping must be exact and result in polygons. The newly created vertices on the clipping plane may not necessarily lie on integer coordinates. Instead, they use the same 4D homogeneous representation that we describe in Section 3.2. It is also not possible to re-triangulate the clipped polygons, as that would require constructing new planes which in general need more bits than the input planes. 

# 4. Evaluation

Our test system has a 4.20 GHz Intel Core i7-7700K with 16 GB RAM. All tests were performed on a single thread. All algorithms were implemented in C++, compiled with Clang 7 using the optimization level -02. Our custom fixed-precision arithmetic was implemented with the help of the _addcarry_u64 and _mulx_u64 intrinsics. For comparing with arbitrary-precision arithmetic we use the gmp [31] library which uses platform-specific optimized assembly code. 

# 4.1. Mathematical foundation

Table 1 shows the performance of the operations presented in Section 3.2. plane_from_points is used when converting a mesh into BSP form and it is expensive because we compute the greatest common divisor (gcd) of the normal components to have them in canonical form. Note that the gcd version is only needed when the normals might otherwise exceed the precision limits (see Fig. 2) which is not the case when the quantization bounds of Section 3.2 are observed. The two important functions during the mesh cutting (and thus the Boolean operations) are intersect_3_planes and classify_vertex. intersect_3_planes constructs the exact intersection position of three planes in homogeneous 4D integer coordinates by computing four $3 \times 3$ determinants. classify_vertex takes such a 4D position and a plane and computes via dot product if the position is on the positive side, the negative side, or exactly on the plane. 

![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/f729c005c02e6935c0c353b6953bd4df5a929d72882e6d944321c9076b91859b.jpg)


# 4.2. Mesh cutting

For evaluating the performance of our mesh cutting and extraction, we measured time, number of vertex constructions, and number of vertex classifications. The results are shown in Fig. 9. Because BSP mesh extraction scales especially poorly with depth, we evaluated on a particularly challenging scenario: a convex object with many faces, corresponding to a purely linear, degenerated BSP tree. Classical mesh cutting (classify all vertices, add cut geometry) scales quadratically with the number of nodes here because the mesh to cut grows linearly more complex and for each cut, all vertices have to be classified (cf. Fig. 9 (e)). Our cutting algorithm with edge descent described in Section 3.3 usually traverses only a fraction of the mesh, making the method efficient even for deep BSPs. Note that the BSP-merging algorithms shown in [1,2,12] work slightly different but use a subroutine to determine how a newly added plane lies in relation to a BSP node's plane and its subhalfspaces. During this procedure the new plane is cut with the bounds of the BSP node's region, which scales linearly with the depth of the node and is equivalent to the complexity of the node's cut-mesh in our approach. This means that [1,2,12] scale like the "naive" baseline in Fig. 9 while our method, at least empirically, scales sub-linear with BSP-depth. 

The other significant performance improvement is caused by the custom-tailored fixed-precision integer arithmetic described in Section 3.2 and consists of two aspects. Firstly, by storing vertex positions as homogeneous 4D integer coordinates, classification can be performed by a simple dot product. Vertex construction requires computing four $3 \times 3$ determinants and is more expensive but also needed less often. Secondly, instead of using arbitrary-precision arithmetic (e.g. gmp), we derived the required precision of all operations and intermediary results and use fixed-precision arithmetic instead. 

In total, we sped up exact mesh extraction by almost two orders of magnitude, allowing us to compute a few million mesh-plane-intersections per second on a single core, even for challenging BSP trees. Note that none of these tests exploit the octree, they only test the underlying BSP algorithms. 

# 4.3. BSP Booleans

Fig. 10 shows the performance of Booleans on random BSPs. The BSPs are generated by sampling random points in a cube, looking up the leaf node at that position, and splitting it along a random direction. Leaf nodes are randomly classified as in or out. We generate two of these BSPs with a size of 1–200 nodes and randomly perform either union, intersection, or difference. As complete sub-BSPs can be culled during merging, performance is more robustly characterized in terms of output complexity. For random BSPs, output complexity is roughly quadratic in input complexity. Real BSPs often only exhibit this quadratic growth close to where the surfaces of the two BSPs meet and otherwise retain their original complexity. Measured on random BSPs and against output complexity, our method can create around 40–50 million BSP nodes per second per CPU core via CSG. This again does not yet use the octree and only tests raw BSP boolean performance. 

# 4.4. Memory consumption

Our octree-embedded BSPs are a persistent data structure. Each Octree node stores some metadata and a BSP. The BSP consists of a flat array of nodes where each node stores index of left and right child and an index into a palette of planes with special indices for leaf nodes. Planes $(a, b, c, d)$ are either $4 + 4 + 4 + 8 = 20$ B or $8 + 8 + 8 + 16 = 40$ B depending on 128 bit or 256 bit arithmetic (plane coefficients are significantly smaller than the largest intermediate result). These planes dominate the persistent memory consumption. The palette can reuse some planes as input triangles might have been split during BSP import, leading to several inner BSP nodes having the same splitting plane. 

During mesh extraction and BSP merging, we temporarily build a half-edge mesh with homogeneous 4D integer positions. In our experiments, this mesh requires around 600–850 B per BSP node on average. In pathological cases it is possible for the mesh to grow quadratically with the BSP size. 

![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/d9b49181e06af38329fd6e4ecd1f5759f7dfae1e4068aaad7c2c603b46711fca.jpg)



(a) two random BSPs and their intersection


When importing a mesh into our data structure we perform polygon clipping where we use a plane palette again. Each input face corresponds to a plane. Our polygons reference a supporting plane and a list of edge planes that define edges and vertices. The clipping process rewrites these references but never creates new planes. Thus, the total temporary space overhead when importing a mesh amounts to less than 50 B per input face. 

# 4.5. Octree

The octree has only a single parameter: the maximal BSP size. To determine which values offer good performance we created the sweep volume shown in Fig. 12 with varying maximal BSP size. The results are shown in Fig. 11. Good choices for the maximal BSP size can clearly be observed in the range of 100 to 200 BSP nodes. 

# 4.6. Stability and correctness

Our core algorithms, BSP mesh extraction and BSP merging, are unconditionally stable and correct. Any BSP with integer planes that satisfy our mild precision limits (see Section 3.2) will work. The reason is that our mesh cutting can provide strong guarantees: Given a strictly convex polyhedral input, the cutting always produces two strictly convex polyhedra corresponding to the positive and negative side of the cutting plane (or the unchanged mesh if the cutting plane does not properly intersect the input). Here, strictly convex means that there are no coplanar faces, i.e. each inner dihedral angle is strictly less than $180^{\circ}$ . Construction and classification of vertex positions are exact due their homogeneous 4D integer representation. These guarantees also extend to the octree which is, in principle, just a union of BSPs. 

The only situations where correctness might get compromised is when importing or exporting floating-point meshes. When our homogeneous 4D integer coordinates are converted into 3D positions of float or double, rounding must occur. While we can provide a result that is correctly rounded to the nearest representable number, this could – in theory – still lead to self-intersecting meshes. In particular, [35] showed that finding a topology-preserving non-intersecting rounding is NP-hard. 

Extra care has to be taken when importing meshes. When the input triangles do not form a closed watertight mesh, the resulting BSP can contain superfluous microgeometry and misclassified cells. The usual workflow is to scale and round the input vertices to half the normal bits to guarantee that all normals can be represented exactly when computing them via cross product. Our most permissive setting uses 256 bit arithmetic with 27 bit positions and 55 bit normals which should be sufficient for any practical purposes, though pathological cases can be constructed. In particular, this guarantees that if a watertight mesh stays self-intersection free under any perturbation with relative magnitude $10^{-8}$ or less then our import finds a BSP corresponding to this mesh within $10^{-8}$ relative Hausdorff distance. 

We ran extensive fuzz tests on random BSPs to test stability and correctness. While we ran out of memory eventually, we were not able to make our algorithm fail. 

# 5. Applications

Our approach excels at iteratively performing a sequence of CSG operations in an asymmetrical setting: Highly complex geometry (the “workpiece”) in the Octree-BSP is subject to many iterative Booleans with a low-complexity Tool-BSP. With that in mind, we implemented applications for sweep-volume calculations (Fig. 12) and a milling simulation (Figs. 13 and 14), as well as reproduced the bunny carving experiment from [1] and the David milling experiment from [3] (Fig. 15). 

For the sweep-volume we took a series of discrete time-steps. Each step, the sweep prism (the tool) of each individual triangle of the input shape is added to the resulting output sweep (the workpiece). A single triangle sweep is computed as the convex hull of the triangles vertices at the current step and the next one. 

The milling simulation was performed in a similar manner. A rotating drill-bit is moved through material with a 30 degree skew. There are 90 four degree steps to achieve a full 360 degree rotation. For each step, the sweep volume is precalculated. After a full rotation, the sweeps can be translated according to the feed rate. In this case, the tool consists of one of the 90, possibly translated, sweep volumes. Then, each of the 90 sweep-volumes are iteratively subtracted from the workpiece, which starts as a cube. This was done for three full rotations, resulting in a total of 270 tool subtractions. We also implemented this simulation with other state-of-the-art methods. Besides our approach, only CGAL with Nef-complexes [13] using exact construction and exact kernel successfully finished the simulation. The results are shown in Fig. 13, the timings and number of resulting faces in Fig. 14. Note that we used the per-step outputs from [3] as inputs for [24] because at the time of writing this, the published source code from [24] only allows the computation of mesh-arrangements but not the resulting Boolean. In order to compute Booleans a classification to determine whether a cell is part of the output is missing. Therefore the computation of the arrangement gives a lower bound to the computation effort. 

![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/91300e63fc0c49a06e56b01d2e5eff292709967c6190d8a5088403579e409788.jpg)



Fig. 12. Sweep of a gear along a Bezier curve with 100 sample positions. This is the result of 764501 union operations and took 281 seconds to create at a maximal BSP size of 300 nodes per octree cell.


# 6. Limitations and future work

Input meshes in floating point formats have to be rounded to suitable integer coordinates before they can be used in our system. While our 256 bit arithmetic presented in Section 3.2 provides nanometer resolution and works for all practical meshes we tested, this is technically a limitation. Replacing our custom-tailored fixed-precision arithmetic with gmp is always possible but causes at least a $10\times$ slowdown. A potential future avenue is to track the precision requirements not per elementary operation (as arbitrary-precision libraries do) but per BSP plane and branch into different fixed-precision routines when constructing or classifying vertices. This will not only remove any precision limitation but might also provide a further speedup when operations can be performed with less bit depth. 

Currently, input meshes are required to be watertight and self-intersection free. While not needed for our use cases, this limitation is easy to lift. Campen and Kobbelt $[2]$ showed how to treat self-intersections as BSP merges and Zhou et al. $[3]$ use winding numbers to correctly classify cells. 

The focus of this work is iterated CSG where a complex object is repeatedly merged with small, relatively simple objects. In such a setting, keeping the octree-embedded BSP as a persistent data structure is essential to achieve high performance: In each step, we only have to compute a series of BSP-BSP booleans where both BSPs have limited complexity. The first is limited due to the subdivision criterion of the octree, the second due to the assumption that the “tool BSP” is simple. Even with our optimizations, large BSP-BSP booleans are still expensive, exhibiting superlinear time complexity. If the task is just to merge two complex objects once, an architecture closer to [2] might be more appropriate: Build an octree containing triangles of the two meshes and only perform the BSP conversion and merging in octree cells containing geometry from both meshes. This would still benefit significantly from our integer computation and BSP merging. 

![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/753ae87e97aa8750b900a33762438f82e01d38c40bfd91436e7f0db60d80bcf9.jpg)



(a) The drill bit.


![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/5e445c314bdd5741f685dc7ee669e495014ed749b99ad6ba87a0be0242e6536e.jpg)



(b) Union of the 90 sweep volumes. Each shade of red is one sweep.


![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/5b1569e986deffa6f694b2630ad056a7ebb2467b8afdc26c322a2bb94f627cbc.jpg)



(c) Result of the milling simulation.


![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/91144c3915a48954d92a0e49083118dcc1514297fd6dfc62ab3ef6ce7391a5c6.jpg)



(d) Wireframe of step 71 of our approach (left) and mesharrangements [3] (right).



Fig. 13. Subtracting the sweep volumes (b) of a rotating drill bit (a). The sweeps were precomputed in 4 degree steps, simulation took 29 seconds with our approach and performed a total of 58472 individual BSP subtractions. Note the large amount of poor-quality triangles in the result of the last successful subtraction using mesh-arrangements [3] (d).


Finally, our current implementation is single-threaded. Especially the octree is perfectly suited for parallelization where each octree cell can be merged in parallel with only minimal synchronization for the occasional octree subdivision and combination. We expect an almost linear speedup in number of cores. 

# 7. Conclusion

In many applications, such as simulations of manufacturing processes, Boolean operations are not only performed once, but repeatedly on a progressively more complex object. For these use cases, we designed a new persistent data structure that we call octree-embedded BSPs. In each cell of a global octree we embed BSP trees with leaves labeled in and out, representing piecewise linear geometry. Inner nodes of the BSP are defined by planes with homogeneous integer coordinates. 


Fig. 14. Milling simulation per-step timings are given in (a), comparing different methods. Cork [30] crashes after 17 steps, both mesh-arrangement approaches [3,24] after 71, and CK10 [2] after 12. Per-step number of faces of the work-piece (b). Our approach produces more faces than cgal-nef [13] due to faces being split across octree cell boundaries.


![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/95db5ade09b3c416b606b92d93bf026881bf5d753c95f81b3b2f77da60fddf50.jpg)



(a)


![](assets/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs/7f6067a4da9884a1c39a6fcac27d549ca25ff533e968669f2f4aacb8775bf819.jpg)



(b)



Fig. 15. The replication of experiments from [1] and [3] using our approach: (a) Subtraction of 10 000 dodecahedra from a block to create the bunny. (b) Subtraction of the sweep volumes of a tool along a path to mill David from a solid block.


We present an exact algorithm for cutting a convex mesh against a plane that remains efficient even for cells in a deep BSP due to traversing only a fraction of the mesh and cutting it in-place. Instead of floating point coordinates we use a plane-based geometry representation and homogeneous 4D integer coordinates with custom-tailored fixed-precision arithmetic to achieve up to 2.5 million mesh-plane cuts per second on a single CPU core. This is used to implement equally exact and efficient CSG and mesh extraction. While the BSPs are used to represent the geometry, the octree is used as an acceleration structure to keep the impact of modifications local and bound the BSP complexity. 

In combination, this results in an unconditionally stable high-performance system for performing iterated CSG of a complex object against many small objects. We demonstrate this by computing sweep volumes and simulating milling where our method outperforms the state of the art both in terms of robustness and performance. 

# Declaration of competing interest

The authors declare that they have no known competing financial interests or personal relationships that could have appeared to influence the work reported in this paper. 

# Funding

This work was supported by the Gottfried-Wilhelm-Leibniz Programme of the Deutsche Forschungsgemeinschaft DFG, by the Excellence Initiative of the German federal and state government and by the European Regional Development Fund within the HDV-Mess project under the funding code EFRE-0500038. 

# References



[1] Bernstein G, Fussell D. Fast, exact, linear booleans. Computer Graphics Forum 2009;28(5):1269–78. 





[2] Campen M, Kobbelt L. Exact and robust (self-)intersections for polygonal meshes. Comput Graph Forum 2010;29(2):397–406. 





[3] Zhou Q, Grinspun E, Zorin D, Jacobson A. Mesh arrangements for solid geometry. ACM Trans Graph 2016;35(4):39:1–39:15. 





[4] Sheng B, Liu B, Li P, Fu H, Ma L, Wu E. Accelerated robust Boolean operations based on hybrid representations. Comput Aided Geom Design 2018;62:133–53. 





[5] Douze M, Franco J-S, Raffin B. QuickCSG: Arbitrary and faster Boolean combinations of n solids. 2015. 





[6] Magalhães SVG, Franklin WR, Andrade MVA. Fast exact parallel 3D mesh intersection algorithm using only orientation predicates. In: Proceedings of the 25th ACM SIGSPATIAL international conference on advances in geographic information systems. New York, NY, USA: ACM; 2017, p. 44:1–44:10. 





[7] Lynn R, Contis D, Hossain M, Huang N, Tucker T, Kurfess T. Voxel model surface offsetting for computer-aided manufacturing using virtualized high-performance computing. J Manuf Syst 2017;43:296–304. 





[8] Sun Y-J, Yan C, Wu S-W, Gong H, Lee C-H. Geometric simulation of 5-axis hybrid additive-subtractive manufacturing based on Tri-dexel model. Int J Adv Manuf Technol 2018;99(9-12):2597-610. 





[9] Barki H, Guennebaud G, Foufou S. Exact, robust, and efficient regularized Booleans on general 3D meshes. Comput Math Appl 2015;70(6):1235–54. 





[10] Lysenko M, D'Souza R, Shene C-K. Improved binary space partition merging. Comput Aided Des 2008;40(12):1113–20. 





[11] Wang CCL, Manocha D. Efficient boundary extraction of BSP solids based on clipping operations. IEEE Trans Vis Comput Graphics 2013;19(1):16–29. 





[12] Naylor B, Amanatides J, Thibault W. Merging BSP trees yields polyhedral set operations. ACM Siggraph Comput Graph 1990;24(4):115–24. 





[13] Hachenberger P, Kettner L, Mehlhorn K. Boolean operations on 3D selective Nef complexes: Data structure, algorithms, optimized implementation and experiments. Comput Geom 2007;38(1):64–99, Special Issue on CGAL. 





[14] Pavić D, Campen M, Kobbelt L. Hybrid Booleans. Comput Graph Forum 2010;29(1):75–87. 





[15] Mei G, Tipper JC. Simple and robust Boolean operations for triangulated surfaces. 2013, CoRR, abs/1308.4434t. URL: http://arxiv.org/abs/1308.4434. arXiv:1308.4434. 





[16] Shewchuk JR. Adaptive precision floating-point arithmetic and fast robust geometric predicates. Discrete Comput Geom 1997;18(3):305–63. 





[17] Nef W. Beiträge zur Theorie der Polyeder: Mit Anwendungen in der Computergraphik. Bern: Herbert Lang; 1978. 





[18] Sugihara K, Iri M. A solid modelling system free from topological inconsistency. J Inf Process 1990;12(4):380–93. 





[19] Wang CCL. Approximate Boolean operations on large polyhedral solids with partial mesh reconstruction. IEEE Trans Vis Comput Graphics 2011;17(6):836–49. 





[20] Schmidt R, Brochu T. Adaptive mesh Booleans. 2016, CoRR, abs/1605.01760. URL: http://arxiv.org/abs/1605.01760. arXiv:1605.01760. 





[21] Feito F, Ogayar C, Segura R, Rivero M. Fast and accurate evaluation of regularized Boolean operations on triangulated solids. Comput Aided Des 2013;45(3):705–16. 





[22] Ogayar-Anguita C, García-Fernández A, Feito-Higueruela F, Segura-Sánchez R. Deferred boundary evaluation of complex CSG models. Adv Eng Softw 2015;85:51–60. 





[23] Jacobson A, Panozzo D, et al. libigl: A simple C++ geometry processing library. 2018, https://libigl.github.io/. 





[24] Cherchi G, Livesu M, Scateni R, Attene M. Fast and robust mesh arrangements using floating-point arithmetic. ACM Trans Graph 2020;39(6). (SIGGRAPH Asia 2020). 





[25] Barill G, Dickson NG, Schmidt R, Levin DI, Jacobson A. Fast winding numbers for soups and clouds. ACM Trans Graph 2018;37(4):43. 





[26] Jense G. Voxel-based methods for CAD. Comput-Aided Design 1989;21(8):528–33. 





[27] Jang D, Kim K, Jung J. Voxel-based virtual multi-axis machining. Int J Adv Manuf Technol 2000;16(10):709–13. 





[28] Hattab A, Taubin G. Interactive fabrication of CSG models with assisted carving. In: Proceedings of the thirteenth international conference on tangible, embedded, and embodied interaction, 2019. p. 677–82. 





[29] Benouamer MO, Michelucci D. Bridging the gap between CSG and Brep via a triple ray representation. In: proceedings of the fourth ACM symposium on solid modeling and applications, 1997. p. 68–79. 





[30] Bernstein G. Cork Boolean library. 2013, https://github.com/gilbo/cork. 





[31] Granlund T, the GMP development team. GNU MP: The GNU multiple precision arithmetic library. 6.1.2 ed.. 2016, http://gmplib.org/. 





[32] Hadamard J. Résolution d'une question relative aux déterminants. Bull Sci Math 1893;17:240–6. 





[33] Gilbert EG, Johnson DW, Keerthi SS. A fast procedure for computing the distance between complex objects in three-dimensional space. IEEE J Robot Autom 1988;4(2):193–203. 





[34] The CGALProject. CGAL user and reference manual. 5.1 ed.. CGAL Editorial Board; 2020, URL: https://doc.cgal.org/5.1/Manual/packages.html. 





[35] Milenkovic V, Nackman LR. Finding compact coordinate representations for polygons and polyhedra. In: Proceedings of the sixth annual symposium on computational geometry. New York, NY, USA: Association for Computing Machinery; 1990, p. 244–52. 

