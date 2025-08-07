Dem Bones Command
=================

The ``demBones`` command exposes Electronic Arts' `Dem Bones` skinning decomposition.

Double Precision Animation Mesh
-------------------------------

Passing the ``-doubleAniMesh`` flag forces the command to instantiate
``DemBonesExt<double, double>`` instead of the default
``DemBonesExt<double, float>``. Using doubles doubles the memory footprint of
animation meshes and can increase runtime due to reduced cache coherence. Use
this mode only when additional precision is required.
