import pytest
import os
import time

import numpy as np

import pye57
from pye57 import libe57
from pye57.utils import get_fields

try:
    from exceptions import WindowsError
except ImportError:
    class WindowsError(OSError):
        pass


def test_hi():
    assert libe57.__doc__


def test_data(*args):
    here = os.path.split(__file__)[0]
    return os.path.join(here, "test_data", *args)


def delete_retry(path):
    try:
        if os.path.exists(path):
            os.remove(path)
    except WindowsError:
        time.sleep(0.1)
        if os.path.exists(path):
            os.remove(path)


@pytest.fixture
def e57_path():
    return test_data("test.e57")


@pytest.fixture
def temp_e57_write(request):
    path = test_data("test_write.e57")
    request.addfinalizer(lambda: delete_retry(path))
    return path


@pytest.fixture
def image_and_points(e57_path):
    f = libe57.ImageFile(e57_path, mode="r")
    scan_0 = libe57.StructureNode(libe57.VectorNode(f.root().get("/data3D")).get(0))
    points = libe57.CompressedVectorNode(scan_0.get("points"))
    return f, points


def test_constants():
    assert libe57.CHECKSUM_POLICY_NONE == 0
    assert libe57.CHECKSUM_POLICY_SPARSE == 25
    assert libe57.CHECKSUM_POLICY_HALF == 50
    assert libe57.CHECKSUM_POLICY_ALL == 100
    assert libe57.E57_INT8_MIN == -128
    assert libe57.E57_INT8_MAX == 127
    assert libe57.E57_INT16_MIN == -32768
    assert libe57.E57_INT16_MAX == 32767
    assert libe57.E57_INT32_MIN == -2147483647 - 1
    assert libe57.E57_INT32_MAX == 2147483647
    assert libe57.E57_INT64_MIN == -9223372036854775807 - 1
    assert libe57.E57_INT64_MAX == 9223372036854775807
    assert libe57.E57_UINT8_MIN == 0
    assert libe57.E57_UINT8_MAX == 255
    assert libe57.E57_UINT16_MIN == 0
    assert libe57.E57_UINT16_MAX == 65535
    assert libe57.E57_UINT32_MIN == 0
    assert libe57.E57_UINT32_MAX == 4294967295
    assert libe57.E57_UINT64_MIN == 0
    assert libe57.E57_UINT64_MAX == 18446744073709551615


def test_open_imagefile(e57_path):
    f = libe57.ImageFile(e57_path, mode="r")
    assert f.isOpen()
    f.close()


def test_open_imagefile_write(temp_e57_write):
    f = libe57.ImageFile(temp_e57_write, mode="w")
    assert f.isOpen()
    f.close()


def test_e57_mode_error(temp_e57_write):
    with pytest.raises(ValueError):
        f = pye57.E57(temp_e57_write, mode="pasta")


def test_get_structure_names(e57_path):
    f = libe57.ImageFile(e57_path, "r")
    root = f.root()
    names = []
    for id_ in range(root.childCount()):
        names.append(root.get(id_).pathName())
    assert names == ['/formatName', '/guid', '/versionMajor', '/versionMinor', '/e57LibraryVersion',
                     '/coordinateMetadata', '/creationDateTime', '/data3D', '/images2D']


def test_get_data3d_nodes(e57_path):
    f = libe57.ImageFile(e57_path, "r")
    root = f.root()
    node = root.get("data3D")
    data3d = libe57.VectorNode(node)
    scan_count = data3d.childCount()
    assert scan_count == 4
    for scan_id in range(scan_count):
        assert isinstance(data3d.get(scan_id), libe57.Node)


def test_get_read_data3d(e57_path):
    f = libe57.ImageFile(e57_path, "r")
    scan_0 = libe57.StructureNode(libe57.VectorNode(f.root().get("/data3D")).get(0))
    points = libe57.CompressedVectorNode(scan_0.get("points"))
    assert points.childCount() == 281300


def test_source_dest_buffers(e57_path):
    f = libe57.ImageFile(e57_path, "r")
    capacity = 1000
    types = list("bBhHlLq?fd")
    sizes = [1, 1, 2, 2, 4, 4, 8, 1, 4, 8]
    buffers = libe57.VectorSourceDestBuffer()
    for t in types:
        data = np.zeros(capacity, t)
        sdb = libe57.SourceDestBuffer(f, "something", data, capacity, True, True)
        buffers.append(sdb)

    for t, sdb, size, in zip(types, buffers, sizes):
        assert sdb.pathName() == "something"
        assert sdb.capacity() == capacity
        assert sdb.stride() == size
        assert sdb.doScaling()
        assert sdb.doConversion()


def test_unsupported_point_field(temp_e57_write):
    with pye57.E57(temp_e57_write, mode="w") as f:
        with pytest.raises(ValueError):
            data = {"cartesianX": np.random.rand(10),
                    "bananas": np.random.rand(10)}
            f.write_scan_raw(data)


def test_source_dest_buffers_raises(e57_path):
    f = libe57.ImageFile(e57_path, "r")
    capacity = 1000
    data = np.zeros(capacity, "i")
    with pytest.raises(ValueError):
        libe57.SourceDestBuffer(f, "something", data, capacity, True, True)


def test_read_points_x(image_and_points):
    imf, points = image_and_points
    bufs = libe57.VectorSourceDestBuffer()
    capacity = 10000
    X = np.zeros(capacity, "f")
    bufs.append(libe57.SourceDestBuffer(imf, "cartesianX", X, capacity, True, True))
    data_reader = points.reader(bufs)
    size = data_reader.read()
    assert size == capacity
    assert not np.all(np.zeros(capacity, "f") == X)


def test_index_out_of_range(e57_path):
    f = libe57.ImageFile(e57_path, "r")
    with pytest.raises(IndexError):
        scan = f.root()["data3D"][-1]
    with pytest.raises(IndexError):
        scan = f.root()["data3D"][5]
    scan_0 = f.root()["data3D"][0]
    with pytest.raises(IndexError):
        r = scan_0["pose"]["rotation"][-1]
    with pytest.raises(IndexError):
        r = scan_0["pose"]["rotation"][5]


def test_read_header(e57_path):
    f = libe57.ImageFile(e57_path, "r")
    data3d = f.root()["data3D"]
    headers = pye57.ScanHeader.from_data3d(data3d)
    fields = ['cartesianX', 'cartesianY', 'cartesianZ', 'intensity', 'rowIndex', 'columnIndex', 'cartesianInvalidState']
    for header in headers:
        assert fields == header.point_fields
    assert headers[0].pretty_print()
    scan_0_rot = [[-0.4443, 0.8958, 0.],
                  [-0.8958, -0.4443, 0.],
                  [0., 0., 1.]]
    assert np.allclose(scan_0_rot, headers[0].rotation_matrix, atol=1e-3)
    scan_0_tra = [301336.23199, 5042597.23676, 15.46649]
    assert np.allclose(scan_0_tra, headers[0].translation)


def test_read_xyz(e57_path):
    e57 = pye57.E57(e57_path)
    xyz = e57.read_scan(0)
    assert np.any(xyz)


def test_read_raw(e57_path):
    e57 = pye57.E57(e57_path)
    header = e57.get_header(0)
    fields = header.point_fields
    data = e57.read_scan_raw(0)
    assert sorted(fields) == sorted(data.keys())
    assert np.any(data["cartesianX"])
    assert len(data["cartesianX"]) == header.point_count


def test_read_write_single_scan(e57_path, temp_e57_write):
    e57 = pye57.E57(e57_path)
    header_source = e57.get_header(0)
    with pye57.E57(temp_e57_write, mode="w") as e57_write:
        raw_data_0 = e57.read_scan_raw(0)
        e57_write.write_scan_raw(raw_data_0, rotation=header_source.rotation, translation=header_source.translation)
    scan_0 = pye57.E57(e57_path).read_scan_raw(0)
    written = pye57.E57(temp_e57_write)
    header = written.get_header(0)
    assert np.allclose(header.rotation, header_source.rotation)
    assert np.allclose(header.translation, header_source.translation)
    scan_0_written = written.read_scan_raw(0)
    fields = "cartesianX cartesianY cartesianZ intensity rowIndex columnIndex cartesianInvalidState".split()
    for field in fields:
        assert np.allclose(scan_0[field], scan_0_written[field])

    scan_0 = e57.read_scan(0)
    scan_0_written = written.read_scan(0)
    for field in scan_0:
        assert np.allclose(scan_0[field], scan_0_written[field])


def test_copy_file(e57_path, temp_e57_write):
    e57 = pye57.E57(e57_path)
    with pye57.E57(temp_e57_write, mode="w") as f:
        for scan_id in range(e57.scan_count):
            header = e57.get_header(scan_id)
            data = e57.read_scan_raw(scan_id)
            f.write_scan_raw(data, scan_header=header)
            header_written = f.get_header(scan_id)
            assert header_written.guid
            assert header_written.temperature == header_written.temperature
            assert header_written.relativeHumidity == header_written.relativeHumidity
            assert header_written.atmosphericPressure == header_written.atmosphericPressure
            assert header_written.rowMinimum == header.rowMinimum
            assert header_written.rowMaximum == header.rowMaximum
            assert header_written.columnMinimum == header.columnMinimum
            assert header_written.columnMaximum == header.columnMaximum
            assert header_written.returnMinimum == header.returnMinimum
            assert header_written.returnMaximum == header.returnMaximum
            assert header_written.intensityMinimum == header.intensityMinimum
            assert header_written.intensityMaximum == header.intensityMaximum
            assert header_written.xMinimum == header.xMinimum
            assert header_written.xMaximum == header.xMaximum
            assert header_written.yMinimum == header.yMinimum
            assert header_written.yMaximum == header.yMaximum
            assert header_written.zMinimum == header.zMinimum
            assert header_written.zMaximum == header.zMaximum
            assert np.allclose(header_written.rotation, header.rotation)
            assert np.allclose(header_written.translation, header.translation)
            assert header_written.acquisitionStart_dateTimeValue == header.acquisitionStart_dateTimeValue
            assert header_written.acquisitionStart_isAtomicClockReferenced == header.acquisitionStart_isAtomicClockReferenced
            assert header_written.acquisitionEnd_dateTimeValue == header.acquisitionEnd_dateTimeValue
            assert header_written.acquisitionEnd_isAtomicClockReferenced == header.acquisitionEnd_isAtomicClockReferenced
            # todo: point groups
            # header.pointGroupingSchemes["groupingByLine"]["idElementName"].value()
            # header.pointGroupingSchemes["groupingByLine"]["groups"]

        assert f.scan_count == e57.scan_count


def test_read_color_absent(e57_path):
    e57 = pye57.E57(e57_path)
    with pytest.raises(ValueError):
        data = e57.read_scan(0, colors=True)


def test_scan_position(e57_path):
    e57 = pye57.E57(e57_path)
    print(e57.scan_position(0))
    assert np.allclose(e57.scan_position(3), np.array([[3.01323456e+05, 5.04260184e+06, 1.56040279e+01]]))


def test_reading():
    file_folder  = 'D:/_data/leica_scanner/' \
        'apb_foyer_01_09_2021/'
        #'holodeck/'
    file_name = \
        'apb-foyer-new'
        #'holodeck'

    file_extention = '.e57'
    f = libe57.ImageFile(file_folder + file_name + file_extention, "r")
    data3d = f.root()["data3D"]
    headers = pye57.ScanHeader.from_data3d(data3d)

    # file = open(file_folder + file_name + '.campose', 'w')
    np_total = 0
    for i in range(0,len(headers)) :
        file = open(file_folder + str(f.root()["data3D"][i][1].value()) + '.campose', 'w')
        file.write('ns 1' + '\n')
        scan_i = f.root()["data3D"][i]['points']
        print('np ' + str(scan_i.childCount()))
        file.write('np ' + str(scan_i.childCount()) + '\n')
        np_total += scan_i.childCount();

        print(headers[i].rotation)
        file.write('r ' + str(headers[i].rotation[0]) + '  ' + str(headers[i].rotation[1]) + '  '
                   + str(headers[i].rotation[2]) + '  ' + str(headers[i].rotation[3]) + '\n')
        if i > 0:
            print(len(headers[i].translation))
            file.write('t ' + str(headers[i].translation[0]) + '  ' + str(headers[i].translation[1]) + '  '
                       + str(headers[i].translation[2]) + '\n')
        else:
            file.write('t 0 0 0\n')
    # file.write('np_total ' + str(np_total) + '\n')
    print(np_total)

test_reading()
