Modified Object Segmentation Database (mOSD)
============================================

This dataset is a modified version of [Object Segmentation Database][osd] (OSD)
and may be used for evaluation of model-free geometrical segmentation
algorithms. The original dataset was collected and released by Richtsfeld et
al. Below is the author's description:

> Segmenting unknown objects from generic scenes is one of the ambitious and
> elusive goals of computer vision. With the recent introduction of cheap and
> powerful 3D sensors (such as the Microsoft Kinect or Asus XtionPRO) which
> deliver a dense point cloud plus color for almost any indoor scene, a renewed
> interest in 3D methods holds the promise to push the envelope slightly
> further.
>
> The Object Segmentation Database provides RGBD data in several subcategories
> to allow evaluation of object segmentation approaches. The database contains
> currently 111 entries:
>
> Learnset:
>   - learn  0-16 => Boxes
>   - learn 17-24 => Stacked Boxes
>   - learn 25-32 => Occluded Objects
>   - learn 33-44 => Cylindric Objects
>
> Testset:
>   - test  0-15 => Boxes
>   - test 16-23 => Stacked Boxes
>   - test 24-30 => Occluded Objects
>   - test 31-42 => Cylindric Objects
>   - test 43-54 => Mixed Objects
>   - test 55-65 => Complex Scene

Modifications
-------------

The modifications are motivated by the need to evaluate model-free geometrical
segmentation algorithms. In a situation when an object is severely occluded, it
may appear as two or more geometrically disjoint segments. A model-free
algorithm can not be expected to know that such segments belong to the same
object. However, the ground truth annotation in the original dataset assigns the
same label to such segments. This leads to undesired penalty of the performance
when evaluating the algorithms in question. The modified annotation assigns
different labels to disjoint segments.

The dataset was changed in a few other ways as well. The list below summarizes
all the modifications that were made to the dataset:

1. Disjoint parts of the same objects were annotated with *different* labels.
2. Labeling was rectified in the vicinity of object edges. The new labels are
   consistent with geometric boundaries of the objects.
3. The point clouds were cropped to remove the floor, the walls, and irrelevant
   stuff on the far end of the table.
4. Z coordinate of invalid points was set to *NaN* instead of 0.

Merging object parts
--------------------

We foresee that like the original dataset, this modified version may be used to
evaluate model-based segmentation methods that are supposed to connect disjoint
parts of the same objects. Therefore, we devised the following label assignment
scheme.

- Label 0: all infinite (*NaN*) points
- Labels 1 to 9: (parts of) the supporting table
- Labels 20 to 29: (parts of) the 1st object
- Labels 30 to 39: (parts of) the 2nd object
- ...

This scheme allows one to obtain new per-object labels by simply changing labels
2–9 to 1, and dividing remaining labels by 10. Indeed, label 0 stays the same,
the parts of the table get label 1, and objects get labels starting from 2.

We provide a script `merge_object_parts.sh` to perform the described relabeling.
The script requires the tool `pcl_convert_pcd_ascii_binary` (distributed as a
part of the Point Cloud Library) to be present in the path.

Authors
=======

Original version
----------------

Dipl.-Ing. Andreas Richtsfeld  
Automation and Control Institute (ACIN)  
Vienna University of Technology  
Gusshausstraße 25-29  
1040 Vienna  
ari(at)acin.tuwien.ac.at

Modified version
----------------

M.Sc. Sergey Alexandrov  
University of Applied Sciences Bonn-Rhein-Sieg  
Grantham-Allee 20  
53757 Sankt Augustin  
sergey.alexandrov(at)h-brs.de

License
=======

The modified version is distributed with permission from Andreas Richtsfeld
under the same license as the original dataset.

[osd]: http://users.acin.tuwien.ac.at/arichtsfeld/?site=4
