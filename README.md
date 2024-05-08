# pointcloud_object_segmenter_lib

Simple shared library for segmenting object from a point cloud using point indices.

Stack:
- OpenCV
- Conan

Install:

```
conan install .
conan build .
```

### Prerequisites
Prerequisites for using package:

* point cloud
* file with point indices of objects (txt)

Example of file with point indices:

```
91870
91871
92509
92510
92511
...
```


## Run tests
Copy test data from folder data to bin
```
cp data/* bin
```

Move to folder bin and run tests:

```
cd bin && ./library_test
```

Result of work:

![ScreenShot](https://raw.github.com/vovaekb/pointcloud_object_segmenter_lib/blob/main/images/object_segmenter_result.png)

Test example of point cloud is located in directory **test**.
