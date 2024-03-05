#include <maya/MFnPlugin.h>
#include <maya/MPxCommand.h>
#include <maya/MArgList.h>
#include <maya/MGlobal.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>
#include <maya/MItSelectionList.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MFnIkJoint.h>
#include <maya/MItMeshVertex.h>
#include <maya/MIntArray.h>
#include <maya/MPointArray.h>
#include <maya/MFloatPointArray.h>
#include <maya/MMatrixArray.h>
#include <maya/MVectorArray.h>
#include <maya/MFnTransform.h>
#include <maya/MFnMatrixData.h>
#include <maya/MAnimControl.h>
#include <maya/MAnimUtil.h>
#include <maya/MFnSkinCluster.h>
#include <maya/MFnSingleIndexedComponent.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MQuaternion.h>

#include <ctime>
#include <vector>
#include <queue>
#include <set>
#include <algorithm>
#include <stack>

#define M_PI            3.14159265358979323846  /* pi */

using namespace std;

long long rays_count = 0;
#pragma region utility

bool fb_have_substring(string in_str, string check_str)
{
    for (int i = 0; i < in_str.size() - check_str.size(); i++)
    {
        string sub_str = in_str.substr(i, check_str.size());
        if (sub_str == check_str)
        {
            return true;
        }
    }
    return false;
}

MString get_detail_name(MDagPath in_path)
{
    MStringArray path_name_array;
    MStringArray name_array;
    in_path.fullPathName().split('|', path_name_array);
    path_name_array[path_name_array.length() - 1].split(':', name_array);
    return name_array[name_array.length() - 1];
}

MDagPathArray get_selected_meshes()
{
    MSelectionList slist;
    MGlobal::getActiveSelectionList(slist);
    MItSelectionList itsl(slist);

    MDagPathArray mesh_paths;
    while (!itsl.isDone())
    {
        MDagPath dagPath;
        MStatus ms = itsl.getDagPath(dagPath);
        itsl.next();
        if (ms != MStatus::kSuccess || !dagPath.isValid())
        {
            continue;
        }
        if (dagPath.apiType() == MFn::kMesh)
        {
            MDagPath::getAPathTo(dagPath.node(), dagPath);
            MFnMesh mesh(dagPath);
            if (!mesh.findPlug("intermediateObject", true).asBool())
            {
                mesh_paths.append(dagPath);
            }
        }
        else if (dagPath.apiType() == MFn::kTransform)
        {
            for (unsigned long c = 0; c < dagPath.childCount(); ++c)
            {
                MObject child = dagPath.child(c);
                if (child.apiType() != MFn::kMesh)
                {
                    continue;
                }
                MDagPath path;
                MDagPath::getAPathTo(child, path);
                MFnMesh mesh(path);
                if (!mesh.findPlug("intermediateObject", true).asBool())
                {
                    mesh_paths.append(path);
                    break;
                }
            }
        }
    }
    return mesh_paths;
}

MDagPathArray get_selected_joints()
{
    MSelectionList slist;
    MGlobal::getActiveSelectionList(slist);
    MItSelectionList itsl(slist);

    MDagPathArray jointPaths;
    while (!itsl.isDone())
    {
        MDagPath dagPath;
        auto ms = itsl.getDagPath(dagPath);
        itsl.next();
        if (ms != MStatus::kSuccess || !dagPath.isValid())
        {
            continue;
        }
        auto apiType = dagPath.apiType();
        if (dagPath.apiType() == MFn::kJoint)
        {
            jointPaths.append(dagPath);
        }
    }
    return jointPaths;
}

MObject find_skin_cluster(
    MDagPath path)
{
    if (path.hasFn(MFn::kMesh))
    {
        MFnMesh mesh(path);
        auto inMeshPlug = mesh.findPlug("inMesh", true);
        MPlugArray srcs;
        inMeshPlug.connectedTo(srcs, true, false);
        for (unsigned long p = 0; p < srcs.length(); ++p)
        {
            auto nobj = srcs[p].node();
            if (nobj.hasFn(MFn::kSkinClusterFilter))
            {
                return nobj;
            }
        }
    }
    else if (path.hasFn(MFn::kTransform))
    {
        MFnTransform jfn(path);
        auto plug = jfn.findPlug("worldMatrix", true);
        plug = plug.elementByLogicalIndex(0);
        MPlugArray dsts;
        plug.connectedTo(dsts, false, true);
        for (unsigned long p = 0; p < dsts.length(); ++p)
        {
            MObject nobj = dsts[p].node();
            if (nobj.hasFn(MFn::kSkinClusterFilter))
            {
                return nobj;
            }
        }
    }
    return MObject::kNullObj;
}

MDagPathArray get_mesh_joints(MDagPathArray meshs)
{
    MDagPathArray joints;
    long offset = 0;
    for (unsigned int m = 0; m < meshs.length(); ++m)
    {
        MObject skinNode = find_skin_cluster(meshs[m]);
        if (skinNode == MObject::kNullObj)
        {
            return MDagPathArray();
        }
    }
    for (unsigned int m = 0; m < meshs.length(); ++m)
    {
        MFnMesh mesh(meshs[m]);
        MObject skinNode = find_skin_cluster(meshs[m]);
        if (skinNode == MObject::kNullObj)
        {
            continue;
        }
        MFnSkinCluster skin(skinNode);
        MIntArray vertexIndices(mesh.numVertices());
        for (int i = 0; i < mesh.numVertices(); ++i)
        {
            vertexIndices[i] = i;
        }
        MFnSingleIndexedComponent singleIndexedComp;
        MObject vertexComp = singleIndexedComp.create(MFn::kMeshVertComponent);
        MDagPathArray influenceObjects;
        skin.influenceObjects(influenceObjects);
        unsigned int numInfDags = influenceObjects.length();
        MIntArray infIndices(numInfDags);
        for (unsigned int i = 0; i < numInfDags; ++i)
        {
            infIndices[i] = i;
            MObject node = influenceObjects[i].node();
            MDagPath path;
            MDagPath::getAPathTo(node, path);
            joints.append(path);
        }
    }
    return joints;
}

MMatrix get_world_matrix(
    MDagPath path)
{
    MFnTransform fnt(path);
    MObject matrixAttr = fnt.attribute("worldMatrix");
    MPlug matrixPlug(path.node(), matrixAttr);
    matrixPlug = matrixPlug.elementByLogicalIndex(0);
    MFnMatrixData matrixData(matrixPlug.asMObject());
    return matrixData.matrix();
}

std::vector<MTime> enumerate_keyframes(
    MDagPathArray joints,
    MTime start = MTime(0.0),
    MTime end = MTime(0.0))
{
    MStatus ms;
    const MString channels[] = {
        "tx", "ty", "tz",
        "rx", "ry", "rz",
        "sx", "sy", "sz"
    };
    std::set<MTime> keyframes;
    for (MDagPath jntPath : joints)
    {
        MFnTransform joint(jntPath);
        for (auto channel : channels)
        {
            MPlug plug = joint.findPlug(channel, true, &ms);
            if (ms != MStatus::kSuccess)
            {
                continue;
            }
            MPlug anim = plug.source(&ms);
            if (ms != MStatus::kSuccess)
            {
                continue;
            }
            MFnAnimCurve curve(anim.node());
            for (unsigned int i = 0; i < curve.numKeys(); ++i)
            {
                MTime frame = curve.time(i);
                if ((start == MTime(0.0) && end == MTime(0.0))
                    || (frame >= start && frame <= end))
                {
                    keyframes.insert(frame);
                }
            }
        }
    }
    return std::vector<MTime>(keyframes.begin(), keyframes.end());
}

MDagPath generate_joint(
    MString name,
    MDagPath parent)
{
    MFnIkJoint rootFn;
    MObject joint = rootFn.create(parent.transform());
    MFnTransform helperFn(joint);
    helperFn.setName(name);
    helperFn.setTranslation(MVector::zero, MSpace::kTransform);
    double sv[3] = { 1, 1, 1 };
    helperFn.setScale(sv);
    helperFn.setRotationQuaternion(0, 0, 0, 1);
    helperFn.setRotateOrientation(MQuaternion::identity, MSpace::kTransform, false);
    return MDagPath::getAPathTo(joint);
}

void copy_joint_animation(
    MDagPath src_joint,
    MDagPath tar_joint,
    MTime anim_first,
    MTime anim_last,
    const long numSamples,
    double transform_effect = 1.0)
{
    const MString attrname[] = {
       "translateX", "translateY", "translateZ",
       "rotateX",    "rotateY",    "rotateZ",
       "scaleX",     "scaleY",     "scaleZ" };
    vector<MObject> attr(9, MObject::kNullObj);
    MFnDagNode node(tar_joint);
    MStatus status = MStatus::kSuccess;

    for (int i = 0; i < 9; ++i)
    {
        MPlug plug = node.findPlug(attrname[i], true);
        if (!MAnimUtil::isAnimated(plug))
        {
            MObject at = node.attribute(attrname[i]);
            MFnAnimCurve acfn(at);
            attr[i] = acfn.create(tar_joint.transform(&status), at, NULL);
            if (status != MStatus::kSuccess)
            {
                string debug = status.errorString().asChar();
            }
        }
        else
        {
            MObjectArray animation;
            for (unsigned int c = 0; c < animation.length(); ++c)
            {
                attr[i] = animation[c];
                if (attr[i].hasFn(MFn::kAnimCurve))
                {
                    break;
                }
            }
        }
    }

    MTime ctime = anim_first;
    for (; ctime <= anim_last; ++ctime)
    {
        MAnimControl::setCurrentTime(ctime);
        MFnTransform helper(src_joint);
        MTransformationMatrix tm = get_world_matrix(src_joint);
        MDagPath parent;

        if (MStatus::kSuccess == MDagPath::getAPathTo(helper.parent(0), parent))
        {

            tm = tm.asMatrix() * get_world_matrix(parent).inverse();
        }

        MVector tv = tm.getTranslation(MSpace::kTransform) * transform_effect;
        status = MFnAnimCurve(attr[0]).addKeyframe(ctime, tv[0]);
        if (status != MStatus::kSuccess)
        {
            string debug = status.errorString().asChar();
        }
        MFnAnimCurve(attr[1]).addKeyframe(ctime, tv[1]);

        MFnAnimCurve(attr[2]).addKeyframe(ctime, tv[2]);
        auto ro = MFnTransform(src_joint).rotationOrder();
        
        double val[6];
        tm.getRotation(val, ro);
        tm.getScale(val + 3, MSpace::kTransform);

        for (long i = 0; i < 6; ++i)
        {
            val[i] *= transform_effect;
            MFnAnimCurve(attr[3 + i]).addKeyframe(ctime, val[i]);
        }
    }
    MAnimControl::setCurrentTime(anim_first);
}

struct joint_node
{
    joint_node(MDagPath in_path) :path(in_path)
    {
        detail_name = get_detail_name(path);
    }
    int index = 0;
    MDagPath path;
    MString detail_name;
    MPoint position = MPoint(0, 0, 0, 0);
};
struct bone_node
{
    int start_joint_index = -1;
    int end_joint_index = -1;
};
struct mapping_node
{
    MVector point;
    double distance = 0;
    int bone_index = -1;
    double weight = 0;
};
struct mapping_result
{
    int vertex_index = -1;
    vector<mapping_node> node_array;
};
struct triangle_weight_index
{
    double weight_distance = 10000;
    long triangle_index = -1;
};
struct raycast_node
{
    MPoint from_point;
    MPoint point;
    int triangle_index = -1;
    float relate_distance = -1;
    double weight = -1;
};
struct raycast_result
{
    vector<raycast_node> raycast_array;
};

void retarget_joints(
    const vector<MString>& src_joints_names,
    const vector<MString>& tar_joints_names,
    vector<long>& out_retarget_result)
{
    out_retarget_result.clear();
    out_retarget_result.assign(tar_joints_names.size(), -1);
    for (int i = 0; i < tar_joints_names.size(); i++)
    {
        for (int j = 0; j < src_joints_names.size(); j++)
        {
            if (tar_joints_names[i] == src_joints_names[j])
            {
                out_retarget_result[i] = j;
            }
        }
    }
}

void unbind_mesh(MDagPath meshPath)
{
    MFnMesh mesh(meshPath);
    MFnDagNode dagNode(mesh.parent(0));
    MString meshName = dagNode.name();
    MGlobal::executeCommand("skinCluster -unbind " + meshName + " -edit");
}

bool ray_triangle_intersection(const MVector orig, const MVector dir, const MVector in_v0, const MVector in_v1, const MVector in_v2, MVector& out_intersectionpoint, double& out_t, double scale = 1)
{
    MVector v0 = in_v0;
    MVector v1 = in_v1;
    MVector v2 = in_v2;
    //scale
    {
        MVector center_tri = (v0 + v1 + v2) / 3;
        v0 = (v0 - center_tri) * scale + center_tri;
        v1 = (v1 - center_tri) * scale + center_tri;
        v2 = (v2 - center_tri) * scale + center_tri;
    }

    const double EPSILON = 0.000000000001;
    //M T
    {
        MVector e1 = v1 - v0;
        MVector e2 = v2 - v0;
        MVector n = e1 ^ e2;
        double ndd = dir * n;
        if (ndd < 0)
        {
            return false;
        }
        MVector h = dir.operator^(e2);
        double a = e1.operator*(h);
        if (a > -EPSILON && a < EPSILON)
        {
            return false; // Ray and triangle are parallel.
        }

        double f = 1.0 / a;
        MVector s = orig - v0;
        double u = s.operator*(h) *f;
        if (u < 0.0 || u > 1.0)
        {
            return false;
        }

        MVector q = s.operator^(e1);
        double v = dir.operator*(q) *f;

        if (v < 0.0 || u + v > 1.0)
        {
            return false;
        }

        double t = e2.operator*(q) *f;
        if (t > EPSILON)
        {
            out_intersectionpoint = orig + dir * t;
            out_t = t;
            return true;
        }
        else
        {
            return false;
        }
    }
}

void triangle_interpolation(
    const MVector v1,
    const MVector v2,
    const MVector v3,
    const MVector p,
    double& w1,
    double& w2,
    double& w3)
{
    MVector x = v3 - v1;
    MVector y = (v2 - v1) ^ x;
    MVector z = y ^ x;
    MMatrix triangle_local_matrix;
    triangle_local_matrix[0][0] = x.x; triangle_local_matrix[0][1] = y.x; triangle_local_matrix[0][2] = z.x; triangle_local_matrix[0][3] = 0;
    triangle_local_matrix[1][0] = x.y; triangle_local_matrix[1][1] = y.y; triangle_local_matrix[1][2] = z.y; triangle_local_matrix[1][3] = 0;
    triangle_local_matrix[2][0] = x.z; triangle_local_matrix[2][1] = y.z; triangle_local_matrix[2][2] = z.z; triangle_local_matrix[2][3] = 0;
    triangle_local_matrix[3][0] = v1.x; triangle_local_matrix[3][1] = v1.y; triangle_local_matrix[3][2] = v1.z; triangle_local_matrix[3][3] = 1;
    MVector tv1 = v1 * triangle_local_matrix;
    MVector tv2 = v2 * triangle_local_matrix;
    MVector tv3 = v3 * triangle_local_matrix;
    MVector tp = p * triangle_local_matrix;
    double Deno = (tv2.z - tv3.z) * (tv1.x - tv3.x) + (tv3.x - tv2.x) * (tv1.z - tv3.z);
    w1 = (tv2.z - tv3.z) * (tp.x - tv3.x) + (tv3.x - tv2.x) * (tp.z - tv3.z);
    w1 /= Deno;
    w2 = (tv3.z - tv1.z) * (tp.x - tv3.x) + (tv1.x - tv3.x) * (tp.z - tv3.z);
    w2 /= Deno;
    w3 = 1 - w1 - w2;
}

void rand_cone_vector(const MVector direction, const double angle_degree, const int N, vector<MVector>& result)
{
    result.clear();
    result.assign(N, MVector());
    srand(time(NULL));
    double cone_angle = angle_degree * M_PI / 180;
    MVector d = direction;
    d.normalize();

    for (int i = 0; i < N; i++)
    {
        double z = ((double)(rand() % N) / N) * (1 - cos(cone_angle)) + cos(cone_angle);
        double phi = ((double)(rand() % N) / N) * 2 * M_PI;
        double x = sqrt(1 - z * z) * cos(phi);
        double y = sqrt(1 - z * z) * sin(phi);


        MVector u = MVector(0, 0, 1) ^ d;
        u.normalize();
        double rot = acos(d * MVector(0, 0, 1));

        MMatrix cross_m;
        cross_m[0][0] = cos(rot) + (1 - cos(rot)) * u.x * u.x; cross_m[0][1] = (1 - cos(rot)) * u.x * u.y - sin(rot) * u.z; cross_m[0][2] = (1 - cos(rot)) * u.x * u.z + sin(rot) * u.y; cross_m[0][3] = 0;
        cross_m[1][0] = (1 - cos(rot)) * u.y * u.x + sin(rot) * u.z; cross_m[1][1] = cos(rot) + (1 - cos(rot)) * u.y * u.y; cross_m[1][2] = (1 - cos(rot)) * u.y * u.z - sin(rot) * u.x; cross_m[1][3] = 0;
        cross_m[2][0] = (1 - cos(rot)) * u.z * u.x - sin(rot) * u.y; cross_m[2][1] = (1 - cos(rot)) * u.z * u.y + sin(rot) * u.x; cross_m[2][2] = cos(rot) + (1 - cos(rot)) * u.z * u.z; cross_m[2][3] = 0;
        cross_m[3][0] = 0; cross_m[3][1] = 0; cross_m[3][2] = 0; cross_m[3][3] = 0;
        MVector rand_dir = cross_m * MVector(x, y, z);
        result[i] = rand_dir;
    }

}

MPointArray compose_pointarray(
    const MDagPath& meshPath)
{
    MPointArray points;
    MFnMesh mesh(meshPath);
    MPointArray pnts;
    mesh.getPoints(pnts, MSpace::kWorld);
    for (unsigned long p = 0; p < pnts.length(); ++p)
    {
        points.append(pnts[p]);
    }
    return points;
}

void compose_weight(
    const MDagPath& meshPath,
    long length,
    unsigned int& numInfs,
    vector<vector<double>>& weight,
    vector<MString>& joints_index_map)
{

    MFnMesh mesh(meshPath);
    MObject skinNode = find_skin_cluster(meshPath);
    if (skinNode == MObject::kNullObj)
    {
        return;
    }

    MFnSkinCluster skin(skinNode);
    MFnSingleIndexedComponent singleIndexedComp;
    MObject vertexComp = singleIndexedComp.create(MFn::kMeshVertComponent);
    MDagPathArray influenceObjects;
    skin.influenceObjects(influenceObjects);
    unsigned int numInfDags = influenceObjects.length();
    MIntArray infIndices(numInfDags);

    numInfs = max(numInfDags, numInfs);
    weight.clear();
    joints_index_map.clear();
    weight.assign(length, vector<double>(numInfs, 0));

    for (unsigned int i = 0; i < numInfDags; ++i)
    {
        infIndices[i] = i;
        MObject node = influenceObjects[i].node();
        MDagPath path;
        MDagPath::getAPathTo(node, path);
        MString joint_name = get_detail_name(influenceObjects[i]);
        joints_index_map.push_back(joint_name);
    }
    MDoubleArray weights(numInfDags * mesh.numVertices(), 0);
    skin.getWeights(meshPath, vertexComp, infIndices, weights);

    for (long v = 0; v < mesh.numVertices(); ++v)
    {
        for (unsigned int i = 0; i < numInfDags; ++i)
        {
            weight[v][i] = weights[v * numInfDags + i];
        }
    }
}

void get_joint_tree(MDagPath in_rootjoint,
    vector<joint_node>& out_joint_group,
    vector<bone_node>& out_bone_group,
    bool except_helperbones = true)
{
    out_joint_group.clear();
    out_bone_group.clear();
    queue<joint_node> joint_queue;


    if (in_rootjoint.apiType() != MFn::kJoint)
    {
        //...
    }
    MFnTransform rootjoint_trans(in_rootjoint);
    joint_node root_joint(in_rootjoint);
    root_joint.index = out_joint_group.size();
    root_joint.position = rootjoint_trans.getTranslation(MSpace::kWorld);//local position
    out_joint_group.push_back(root_joint);

    joint_queue.push(root_joint);
    while (!joint_queue.empty())
    {
        MDagPath dagpath_ptr = joint_queue.front().path;

        string debug_str = dagpath_ptr.fullPathName().asChar();
        int child_number = dagpath_ptr.childCount();
        for (int i = 0; i < child_number; i++)
        {
            auto child_node = dagpath_ptr.child(i);
            if (child_node.apiType() != MFn::kJoint)
            {
                continue;
            }
            MDagPath child_path;
            MDagPath::getAPathTo(child_node, child_path);
            debug_str = child_path.fullPathName().asChar();
            if (except_helperbones && fb_have_substring(child_path.partialPathName().asChar(), "helper"))
            {
                continue;
            }

            MFnTransform joint_trans(child_path);
            joint_node current_joint(child_path);
            bone_node current_bone;
            current_bone.start_joint_index = joint_queue.front().index;
            current_bone.end_joint_index = out_joint_group.size();

            current_joint.index = out_joint_group.size();
            current_joint.position = joint_trans.getTranslation(MSpace::kWorld);
            out_bone_group.push_back(current_bone);
            out_joint_group.push_back(current_joint);
            joint_queue.push(current_joint);
        }

        joint_queue.pop();
    }
}

void copy_joint_tree(
    MDagPath root_joint,
    MDagPath copy_joint,
    MTime anim_first,
    MTime anim_last,
    const long numSamples,
    double transform_effect = 1.0)
{
    MStringArray joint_path_names;
    copy_joint.fullPathName().split('|', joint_path_names);
    // generate new joint with full path name
    MDagPath tar_joint = generate_joint(joint_path_names[joint_path_names.length() - 1], root_joint);
    string debug_str = tar_joint.fullPathName().asChar();

    copy_joint_animation(copy_joint, tar_joint, anim_first, anim_last, numSamples, transform_effect);

    debug_str = copy_joint.fullPathName().asChar();
    int child_number = copy_joint.childCount();
    for (int i = 0; i < child_number; i++)
    {
        auto child_node = copy_joint.child(i);
        if (child_node.apiType() != MFn::kJoint)
        {
            continue;
        }
        MDagPath child_path;
        MDagPath::getAPathTo(child_node, child_path);
        debug_str = child_path.fullPathName().asChar();
        if (fb_have_substring(child_path.partialPathName().asChar(), "helper"))
        {
            //copy child trees
            copy_joint_tree(tar_joint, child_path, anim_first, anim_last, numSamples);
        }
    }
}

void hb_set_weight(
    MDagPath& meshPath,
    MDagPath& rootjointPath,
    unsigned int numInfs,
    const vector<vector<double>>& old_tar_weight,
    const vector<vector<double>>& result_weight,
    const vector<MString>& src_joints_name,
    vector<MString>& tar_joints_name,
    const double blend_default_weight)
{

    MSelectionList sl, psl;
    MGlobal::getActiveSelectionList(psl);

    vector<joint_node> tar_joint_group;
    vector<bone_node> useless_gourp;
    get_joint_tree(rootjointPath, tar_joint_group, useless_gourp, false);

    for (int i = 0; i < tar_joint_group.size(); i++)
    {
        sl.add(tar_joint_group[i].path);
    }

    MFnMesh mesh(meshPath);
    MFnDagNode dagNode(mesh.parent(0));
    MString meshName = dagNode.name();
    const long numVerts = mesh.numVertices();

    //rebind
    MGlobal::executeCommand("skinCluster -unbind " + meshName + " -edit");
    char bufCmd[256];
    sprintf(bufCmd, "skinCluster -maximumInfluences %d -toSelectedBones ", numInfs);
    MSelectionList asl(sl);
    asl.add(meshPath);
    MGlobal::setActiveSelectionList(asl);
    MGlobal::executeCommand(MString(bufCmd) + "-name " + meshName + "Cluster");

    //get skin cluster
    MObject skinObj = find_skin_cluster(meshPath);
    if (skinObj == MObject::kNullObj)
    {
        return;
    }
    MFnSkinCluster skin(skinObj);
    MIntArray vertexIndices(numVerts, 0);
    for (long i = 0; i < numVerts; ++i)
    {
        vertexIndices[i] = i;
    }
    MFnSingleIndexedComponent singleIndexedComp;
    auto vertexComp = singleIndexedComp.create(MFn::kMeshVertComponent);
    singleIndexedComp.addElements(vertexIndices);
    MDagPathArray infDags;
    const long numInfDags = skin.influenceObjects(infDags);

    //Get joint-name map
    vector<MString> result_joints_name;
    for (unsigned int i = 0; i < numInfDags; ++i)
    {
        MString joint_name = get_detail_name(infDags[i]);
        result_joints_name.push_back(joint_name);
    }

    //Align joint-name map
    vector<long> result_src_joints_retarget;
    vector<long> result_tar_joints_retarget;
    retarget_joints(src_joints_name, result_joints_name, result_src_joints_retarget);
    retarget_joints(tar_joints_name, result_joints_name, result_tar_joints_retarget);

    MIntArray infIndices(numInfDags, 0);
    for (long i = 0; i < numInfDags; ++i)
    {
        infIndices[i] = i;
    }
    MDoubleArray weights(numVerts * numInfDags, 0.0);
    skin.getWeights(meshPath, vertexComp, infIndices, weights);
    for (long v = 0; v < numVerts; ++v)
    {
        for (long i = 0; i < numInfDags; ++i)
        {
            double old_w = 0;
            if (i < result_tar_joints_retarget.size() && result_tar_joints_retarget[i] != -1)
            {
                old_w = old_tar_weight[v][result_tar_joints_retarget[i]];
            }
            double result_w = 0;
            if (i < result_src_joints_retarget.size() && result_src_joints_retarget[i] != -1)
            {
                result_w = result_weight[v][result_src_joints_retarget[i]];
            }
            weights[v * numInfDags + i] = (1.0 - blend_default_weight) * old_w + blend_default_weight * result_w;
        }
    }
    skin.setWeights(meshPath, vertexComp, infIndices, weights);

    MGlobal::setActiveSelectionList(psl);
}

void copy_helper_joints(
    MDagPath in_src_rootjoint,
    MDagPath in_out_tar_rootjoint,
    MTime anim_first,
    MTime anim_last,
    const long numSamples,
    double transform_effect = 1.0)
{
    vector<joint_node> src_joint_group;
    vector<joint_node> tar_joint_group;
    vector<bone_node> useless_gourp;
    //first get the main joints
    get_joint_tree(in_src_rootjoint, src_joint_group, useless_gourp);
    get_joint_tree(in_out_tar_rootjoint, tar_joint_group, useless_gourp);

    if (src_joint_group.size() != tar_joint_group.size())
    {
        //...
    }
    //Check each main joint
    //Copy if there are helper joints
    for (int i = 0; i < src_joint_group.size(); i++)
    {
        MDagPath dagpath_ptr = src_joint_group[i].path;
        string debug_str = dagpath_ptr.fullPathName().asChar();
        int child_number = dagpath_ptr.childCount();
        for (int j = 0; j < child_number; j++)
        {
            auto child_node = dagpath_ptr.child(j);
            if (child_node.apiType() != MFn::kJoint)
            {
                continue;
            }
            MDagPath child_path;
            MDagPath::getAPathTo(child_node, child_path);
            debug_str = child_path.fullPathName().asChar();
            if (fb_have_substring(child_path.partialPathName().asChar(), "helper"))
            {
                //copy all helper joints
                copy_joint_tree(tar_joint_group[i].path, child_path, anim_first, anim_last, numSamples, transform_effect);
            }
        }
    }
}


void get_weight_distance(
    const MIntArray& src_index,
    long src_numtri,
    const vector<vector<double>>& src_weight,
    const vector<vector<double>>& tar_weight,
    const vector<long>& tar_joints_retarget,
    vector<vector<triangle_weight_index>>& tar_best_weight_triangle_map)
{
    long num_inf = src_weight[0].size();
    long tar_num_vertex = tar_weight.size();
    vector<vector<double>> src_triangle_weight;
    src_triangle_weight.assign(src_numtri, vector<double>(num_inf, 0));
    //Reference triangle average weight
    for (long i = 0; i < src_numtri; i++)
    {
        for (long j = 0; j < num_inf; j++)
        {
            long tri_vertex_0 = src_index[i * 3 + 0];
            long tri_vertex_1 = src_index[i * 3 + 1];
            long tri_vertex_2 = src_index[i * 3 + 2];
            src_triangle_weight[i][j] += src_weight[tri_vertex_0][j];
            src_triangle_weight[i][j] += src_weight[tri_vertex_1][j];
            src_triangle_weight[i][j] += src_weight[tri_vertex_2][j];
            src_triangle_weight[i][j] /= 3.0;
        }
    }
    //Calculate weighted distance
    tar_best_weight_triangle_map.clear();
    tar_best_weight_triangle_map.assign(tar_num_vertex, vector<triangle_weight_index>(src_numtri, triangle_weight_index()));
    for (long i = 0; i < tar_num_vertex; i++)
    {
        for (long j = 0; j < src_numtri; j++)
        {
            double weight_distance = 0;
            for (long l = 0; l < num_inf; l++)
            {
                // == -1 Means that the reference is missing the weight range of this joint and is not used as a distance difference.
                if (l < tar_joints_retarget.size() && tar_joints_retarget[l] != -1)
                {

                    double t_w = tar_weight[i][l];
                    double s_w = src_triangle_weight[j][tar_joints_retarget[l]];
                    //manhattan distance
                    //weight_distance += abs(t_w - s_w);
                    //Euclidean distance
                    weight_distance += pow(t_w - s_w, 2.0);
                }
            }
            //Euclidean distance
            weight_distance = pow(weight_distance, 0.5);
            tar_best_weight_triangle_map[i][j].triangle_index = j;
            tar_best_weight_triangle_map[i][j].weight_distance = weight_distance;
        }
    }
}

void get_mapping_points_geodesic_voxel(
    const MPointArray& in_points,
    const vector<joint_node>& in_joint_group,
    const vector<bone_node>& in_bone_group,
    const vector<vector<double>>& bones_weight,
    const vector<MString>& bones_weight_names,
    double max_distance,
    vector<mapping_result>& out_result_mapping_points)
{
    //Matching weights and joint trees
    vector<int> bones_weight_index;
    bones_weight_index.assign(in_joint_group.size(), -1);
    for (int i = 0; i < in_joint_group.size(); i++)
    {
        for (int j = 0; j < bones_weight_names.size(); j++)
        {
            if (bones_weight_names[j] == in_joint_group[i].detail_name)
            {
                bones_weight_index[i] = j;
                break;
            }
        }
    }

    out_result_mapping_points.clear();
    out_result_mapping_points.assign(in_points.length(), mapping_result());
#pragma omp parallel for
    for (int vert_it = 0; vert_it < in_points.length(); vert_it++)
    {
        auto current_vert = in_points[vert_it];
        for (int bone_it = 0; bone_it < in_bone_group.size(); bone_it++)
        {
            auto current_bone = in_bone_group[bone_it];
            auto start_joint = in_joint_group[current_bone.start_joint_index];
            auto end_joint = in_joint_group[current_bone.end_joint_index];
            long weight_index = bones_weight_index[current_bone.start_joint_index];
            //If it does not have a weight, it will not be mapped.
            if (weight_index < 0 || bones_weight[vert_it][weight_index] < 1.0e-5)
            {
                continue;
            }


            MVector start_point = start_joint.position;
            MVector end_point = end_joint.position;
            MVector bone_vector = end_point - start_point;
            MVector normalize_bone_vector = bone_vector;
            normalize_bone_vector.normalize();
            MVector w = current_vert - start_point;
            MVector p = (w * normalize_bone_vector) * normalize_bone_vector + start_point; // Projection point = projection length * bone unit vector L + bone starting point vector (global local coordinates)
            //Compare to get the best projection point
            //distance£¬starting point - Projection point > max_dis 
            //distance£¬Projection point - end point > max_dis
            MVector check_direction = p - start_point;
            bool left_is_legal = check_direction * normalize_bone_vector > 0; 
            left_is_legal |= max_distance > check_direction.length(); 
            check_direction = p - end_point;
            bool right_is_legal = check_direction * normalize_bone_vector < 0;
            right_is_legal |= max_distance > check_direction.length();
            double distance = (p - current_vert).length();
            if (right_is_legal && left_is_legal)
            {
                mapping_node node;
                node.point = p;
                node.bone_index = bone_it;
                node.distance = distance;
                node.weight = bones_weight[vert_it][weight_index];
                out_result_mapping_points[vert_it].node_array.push_back(node);
            }
            else //Find the closest point of this bone
            {
                //At this time, the closest point can only be the starting point or the end point
                double start_distance = (start_point - current_vert).length();
                double end_distance = (end_point - current_vert).length();
                if (start_distance < end_distance)
                {
                    mapping_node node;
                    node.point = start_point;
                    node.bone_index = bone_it;
                    node.distance = start_distance;
                    node.weight = bones_weight[vert_it][weight_index];
                    out_result_mapping_points[vert_it].node_array.push_back(node);
                }
                else if (end_distance < start_distance)
                {
                    mapping_node node;
                    node.point = end_point;
                    node.bone_index = bone_it;
                    node.distance = end_distance;
                    node.weight = bones_weight[vert_it][weight_index];
                    out_result_mapping_points[vert_it].node_array.push_back(node);
                }
            }
        }
        out_result_mapping_points[vert_it].vertex_index = vert_it;
    }
}

void get_correspondence_points(
    const MPointArray& src_default_points,
    const MIntArray& src_indeice,
    const long src_num_triangles,
    const MPointArray& tar_points,
    const vector<mapping_result>& tar_mapping_points,
    const vector<joint_node>& src_joint_group,
    const vector<joint_node>& tar_joint_group,
    const vector<bone_node>& src_bone_group,
    const vector<bone_node>& tar_bone_group,
    const vector<vector<triangle_weight_index>>& tar_best_weight_triangle_map,
    const int sample_number,
    const double sample_degree,
    vector<raycast_result>& raycast_result_array)
{
    raycast_result_array.clear();
    raycast_result_array.assign(tar_mapping_points.size(), raycast_result());

    //Match reference and target joint trees tar->src
    vector<int> src_joint_index;
    src_joint_index.assign(tar_joint_group.size(), -1);
    for (int i = 0; i < tar_joint_group.size(); i++)
    {
        for (int j = 0; j < src_joint_group.size(); j++)
        {
            if (src_joint_group[j].detail_name == tar_joint_group[i].detail_name)
            {
                src_joint_index[i] = j;
                break;
            }
        }
    }



    //Traverse each target point and find the nearest source point according to the joint topology.
#pragma omp parallel for
    for (int current_vert = 0; current_vert < tar_mapping_points.size(); current_vert++)
    {
        for (int current_node = 0; current_node < tar_mapping_points[current_vert].node_array.size(); current_node++)
        {
            auto current_tar_bone_index = tar_mapping_points[current_vert].node_array[current_node].bone_index;
            auto current_node_weight = tar_mapping_points[current_vert].node_array[current_node].weight;
            //tar pv
            MVector curren_tar_v = tar_points[tar_mapping_points[current_vert].vertex_index];
            MVector current_tar_p = tar_mapping_points[current_vert].node_array[current_node].point;//local coordinate
            MVector current_tar_pv = curren_tar_v - current_tar_p;
            MVector current_tar_normal_pv = current_tar_pv;
            current_tar_normal_pv.normalize();
            int tar_start_joint_index = tar_bone_group[current_tar_bone_index].start_joint_index;
            int tar_end_joint_index = tar_bone_group[current_tar_bone_index].end_joint_index;
            MVector current_tar_bone_start_point = tar_joint_group[tar_start_joint_index].position;
            MVector current_tar_bone_end_point = tar_joint_group[tar_end_joint_index].position;
            MVector current_tar_bone_v = current_tar_bone_end_point - current_tar_bone_start_point;
            double tar_distance = (current_tar_p - current_tar_bone_start_point).length() / current_tar_bone_v.length();//target map point percentage

            //src bone vtr
            int src_start_joint_index = src_joint_index[tar_start_joint_index];
            int src_end_joint_index = src_joint_index[tar_end_joint_index];
            MVector current_src_bone_start_point = src_joint_group[src_start_joint_index].position;
            MVector current_src_bone_end_point = src_joint_group[src_end_joint_index].position;
            MVector current_src_bone_v = current_src_bone_end_point - current_src_bone_start_point;
            MVector current_src_bone_normal_v = current_src_bone_v;
            current_src_bone_normal_v.normalize();

            //on the bone
            MVector two_bone_normal = current_src_bone_v ^ current_tar_bone_v;
            two_bone_normal.normalize();
            double theta = current_src_bone_v.angle(current_tar_bone_v);
            double cos_theta = cos(theta);
            double sin_theta = sin(theta);
            //Intersection judgment
            {
                MVector p = current_src_bone_start_point + current_src_bone_v * tar_distance;
                MVector d = current_tar_normal_pv;
                //MVector d = (current_tar_normal_pv * cos_theta) + ((two_bone_normal ^ current_tar_normal_pv) * sin_theta) + (two_bone_normal * (two_bone_normal* current_tar_normal_pv)) * (1 - cos_theta);
                d.normalize();

                MVector best_q;
                double range = 1;
                int sample_n = sample_number;//* current_node_weight;
                vector<MVector> sample_direction_array;
                rand_cone_vector(d, sample_degree, sample_n, sample_direction_array);
                rays_count += sample_n;
                for (int s = 0; s < sample_n; s++)
                {
                    MVector sample_direction = sample_direction_array[s];
                    double best_distance = 10000000;
                    bool has_intersection = false;
                    raycast_node best_result_node;
                    for (int i = 0; i < src_num_triangles; i++)
                    {
                        long src_tri_index = tar_best_weight_triangle_map[current_vert][i].triangle_index;
                        double weight_distance = tar_best_weight_triangle_map[current_vert][i].weight_distance;
                        MVector v0 = src_default_points[src_indeice[src_tri_index * 3 + 0]] * range;
                        MVector v1 = src_default_points[src_indeice[src_tri_index * 3 + 1]] * range;
                        MVector v2 = src_default_points[src_indeice[src_tri_index * 3 + 2]] * range;
                        MVector q;
                        double t = 0;

                        if (ray_triangle_intersection(p, sample_direction, v0, v1, v2, q, t, 1.0))
                        {
                            MVector center_tri = (v0 + v1 + v2) / 3;
                            double distance = (center_tri - q).length() + t;
                            //distance += weight_distance;
                            if (distance < best_distance)
                            {
                                best_distance = distance;
                                has_intersection = true;
                                best_result_node.from_point = p;
                                best_result_node.point = q;
                                best_result_node.triangle_index = src_tri_index;
                                best_result_node.weight = current_node_weight;
                                best_result_node.relate_distance = current_tar_pv.length() / t;
                            }
                        }


                    }
                    if (has_intersection)
                    {
                        raycast_result_array[current_vert].raycast_array.push_back(best_result_node);
                    }
                }
            }
        }
    }
}

void weight_transform(
    const MPointArray& src_default_points,
    const vector<long>& tar_joints_retarget,
    const vector<vector<double>>& src_default_weight,
    const vector<vector<double>>& src_deform_weight,
    const vector<vector<double>>& tar_weight,
    const MIntArray& src_indeice,
    const vector<mapping_result>& tar_mapping_points,
    const vector<raycast_result>& raycast_result_array,
    const double weight_decay,
    vector<vector<double>>& result_weight)
{
    int inf_number = result_weight[0].size();
#pragma omp parallel for
    for (int current_vert = 0; current_vert < tar_mapping_points.size(); current_vert++)
    {
        //Interpolate weights according to position proportion (the deformed model is copied)
        double sum_weight = 0;
        vector<double> current_weight(inf_number, 0);//in order of reference
        int tar_index = tar_mapping_points[current_vert].vertex_index;
        for (int current_raycast_node = 0; current_raycast_node < raycast_result_array[current_vert].raycast_array.size(); current_raycast_node++)
        {
            int triangle_index = raycast_result_array[current_vert].raycast_array[current_raycast_node].triangle_index;
            int src_p0_index = src_indeice[triangle_index * 3 + 0];
            int src_p1_index = src_indeice[triangle_index * 3 + 1];
            int src_p2_index = src_indeice[triangle_index * 3 + 2];
            MPoint best_q = raycast_result_array[current_vert].raycast_array[current_raycast_node].point;
            double weight = raycast_result_array[current_vert].raycast_array[current_raycast_node].weight;
            //Find the three points of the triangle (interpolate using the undeformed model)
            MPointArray default_triangle;
            default_triangle.append(src_default_points[src_p0_index]);
            default_triangle.append(src_default_points[src_p1_index]);
            default_triangle.append(src_default_points[src_p2_index]);

            //center of gravity ratio
            double qtop0 = 0;
            double qtop1 = 0;
            double qtop2 = 0;
            triangle_interpolation(default_triangle[0], default_triangle[1], default_triangle[2], best_q, qtop0, qtop1, qtop2);

            //weight mask
            int match_count = 0;
            double match_weight = 0;
            for (int inf_index = 0; inf_index < tar_joints_retarget.size(); inf_index++)
            {
                if (tar_joints_retarget[inf_index] == -1)
                {
                    //Means that the reference is missing the weight range of this joint. Not used.
                    continue;
                }
                double tar_w = tar_weight[current_vert][inf_index];
                double src_df_w0 = src_default_weight[src_p0_index][tar_joints_retarget[inf_index]];
                double src_df_w1 = src_default_weight[src_p1_index][tar_joints_retarget[inf_index]];
                double src_df_w2 = src_default_weight[src_p2_index][tar_joints_retarget[inf_index]];
                double src_df_qw = src_df_w0 * qtop0 + src_df_w1 * qtop1 + src_df_w2 * qtop2;
                //average
                if (tar_w + src_df_qw > 0)
                {
                    match_weight += abs(tar_w - src_df_qw) / (tar_w + src_df_qw);
                    match_count++;
                }
            }
            //Normalization + exponential decay
            match_weight /= match_count;
            match_weight = (1 - match_weight);
            match_weight = pow(match_weight, weight_decay);

            match_weight *= weight;

            for (int inf_index = 0; inf_index < inf_number; inf_index++)
            {
                current_weight[inf_index] += src_deform_weight[src_p0_index][inf_index] * qtop0 * match_weight;
                current_weight[inf_index] += src_deform_weight[src_p1_index][inf_index] * qtop1 * match_weight;
                current_weight[inf_index] += src_deform_weight[src_p2_index][inf_index] * qtop2 * match_weight;
            }
        }

        for (int inf_index = 0; inf_index < inf_number; inf_index++)
        {
            sum_weight += current_weight[inf_index];
        }
        //Normalization
        for (int inf_index = 0; inf_index < inf_number; inf_index++)
        {
            if (sum_weight != 0)
            {
                current_weight[inf_index] /= sum_weight;
                //in order of reference
                result_weight[tar_index][inf_index] = current_weight[inf_index];
            }
        }
    }
}

#pragma endregion

class deformation_transfer
{

public:

    void rig_reflection_geodesic_voxel(
        MDagPath src_default_rootjoint,
        MDagPath tar_rootjoint,
        const MPointArray& src_default_vertex,
        const MPointArray& tar_vertex,
        const MIntArray& src_index,
        long src_numtri,
        const vector<vector<double>>& src_default_weight,
        const vector<vector<double>>& tar_weight,
        const vector<MString>& tar_joints_name,
        const vector<vector<triangle_weight_index>>& tar_best_weight_triangle_map,
        const int sample_number,
        const double sample_degree,
        vector<mapping_result>& tar_mapping_points,
        vector<raycast_result>& raycast_result_array,
        const int debug_state
    )
    {
        //Depth traversal records bones
        vector<joint_node> src_default_joint_group;
        vector<joint_node> tar_joint_group;
        vector<bone_node> src_default_bone_group;
        vector<bone_node> tar_bone_group;
        get_joint_tree(src_default_rootjoint, src_default_joint_group, src_default_bone_group);
        get_joint_tree(tar_rootjoint, tar_joint_group, tar_bone_group);

        //Requires the same joint topology
        if (src_default_bone_group.size() != tar_bone_group.size())
        {
            //...
        }
        get_mapping_points_geodesic_voxel(
            tar_vertex,
            tar_joint_group,
            tar_bone_group,
            tar_weight,
            tar_joints_name,
            0, tar_mapping_points);

        //Debug
        if (debug_state == 1)
        {
            MPointArray debug_points;
            MIntArray debug_polygoncounts;
            MIntArray debug_polygonconnects;
            for (int i = 0; i < tar_mapping_points.size(); i++)
            {
                for (int j = 0; j < tar_mapping_points[i].node_array.size(); j++)
                {
                    debug_points.append(tar_mapping_points[i].node_array[j].point);
                    debug_points.append(tar_mapping_points[i].node_array[j].point);
                    debug_points.append(tar_mapping_points[i].node_array[j].point);
                    debug_points.append(tar_mapping_points[i].node_array[j].point);
                    debug_polygoncounts.append(4);
                    debug_polygonconnects.append(debug_points.length() - 4);
                    debug_polygonconnects.append(debug_points.length() - 3);
                    debug_polygonconnects.append(debug_points.length() - 2);
                    debug_polygonconnects.append(debug_points.length() - 1);
                }
            }
            MStatus status;
            MFnMesh outMeshFn;
            outMeshFn.create(
                debug_points.length(),
                debug_polygoncounts.length(),
                debug_points,
                debug_polygoncounts,
                debug_polygonconnects,
                MObject::kNullObj,
                &status);
        }

        //Debug
        if (debug_state == 1)
        {
            MPointArray debug_points;
            MIntArray debug_polygoncounts;
            MIntArray debug_polygonconnects;
            for (int i = 0; i < tar_mapping_points.size(); i++)
            {
                for (int j = 0; j < tar_mapping_points[i].node_array.size(); j++)
                {
                    debug_points.append(tar_mapping_points[i].node_array[j].point);
                    debug_points.append(tar_mapping_points[i].node_array[j].point);
                    debug_points.append(tar_vertex[i]);
                    debug_points.append(tar_vertex[i]);
                    debug_polygoncounts.append(4);
                    debug_polygonconnects.append(debug_points.length() - 4);
                    debug_polygonconnects.append(debug_points.length() - 3);
                    debug_polygonconnects.append(debug_points.length() - 2);
                    debug_polygonconnects.append(debug_points.length() - 1);
                }
            }
            MStatus status;
            MFnMesh outMeshFn;
            outMeshFn.create(
                debug_points.length(),
                debug_polygoncounts.length(),
                debug_points,
                debug_polygoncounts,
                debug_polygonconnects,
                MObject::kNullObj,
                &status);
        }

        MPointArray src_default_normalize_points = src_default_vertex;
        MPointArray tar_normalize_points = tar_vertex;
        //Traverse each point, find the nearest line segment, and obtain the vector
        //Calculate the distance according to the ratio of the unit vector and the projection point to obtain the projection point on the bone
        //Back-project onto the undeformed skin of the source model
        get_correspondence_points(
            src_default_normalize_points,
            src_index,
            src_numtri,
            tar_normalize_points,
            tar_mapping_points,
            src_default_joint_group,
            tar_joint_group,
            src_default_bone_group,
            tar_bone_group,
            tar_best_weight_triangle_map,
            sample_number,
            sample_degree,
            raycast_result_array);

        //Debug
        if (debug_state == 1)
        {
            MPointArray debug_points;
            MIntArray debug_polygoncounts;
            MIntArray debug_polygonconnects;
            for (int i = 0; i < tar_mapping_points.size(); i++)
            {
                for (int j = 0; j < raycast_result_array[i].raycast_array.size(); j++)
                {
                    debug_points.append(raycast_result_array[i].raycast_array[j].from_point);
                    debug_points.append(raycast_result_array[i].raycast_array[j].from_point);
                    debug_points.append(raycast_result_array[i].raycast_array[j].point);
                    debug_points.append(raycast_result_array[i].raycast_array[j].point);
                    debug_polygoncounts.append(4);
                    debug_polygonconnects.append(debug_points.length() - 4);
                    debug_polygonconnects.append(debug_points.length() - 3);
                    debug_polygonconnects.append(debug_points.length() - 2);
                    debug_polygonconnects.append(debug_points.length() - 1);
                }
            }
            MStatus status;
            MFnMesh outMeshFn;
            outMeshFn.create(
                debug_points.length(),
                debug_polygoncounts.length(),
                debug_points,
                debug_polygoncounts,
                debug_polygonconnects,
                MObject::kNullObj,
                &status);
        }
    }

    void raycast_weight_transform(
        int sample_number,
        double sample_degree,
        double weight_decay,
        double blend_default_weight,
        int debug_state)
    {
        //Get meshes and joints
        auto dpa = get_selected_meshes();
        if (dpa.length() != 3)
        {
            MGlobal::displayError("Select default,deform source and destination meshs");
            return;
        }
        mesh_joint_paths = get_mesh_joints(dpa);
        if (mesh_joint_paths.length() == 0)
        {
            MGlobal::displayError("Select source joint");
            return;
        }
        auto joint_dpa = get_selected_joints();
        if (joint_dpa.length() != 3)
        {
            MGlobal::displayError("Select default,deform source and destination root joint");
            return;
        }
        string Debug_j0 = joint_dpa[0].fullPathName().asChar();
        string Debug_j1 = joint_dpa[1].fullPathName().asChar();
        root_joint_paths[0] = joint_dpa[0];
        root_joint_paths[1] = joint_dpa[1];
        root_joint_paths[2] = joint_dpa[2];


        // Target mesh data init
        // connectivity info
        MFnMesh tarMesh(dpa[2]);
        long numVerts = tarMesh.numVertices();
        MIntArray triCounts;
        tarMesh.getTriangles(triCounts, tri_verts);
        long numTris = 0;
        for (unsigned int f = 0; f < triCounts.length(); ++f)
        {
            for (long i = 0; i < triCounts[f]; ++i, ++numTris)
            {
                long vi1 = tri_verts[numTris * 3 + 0];
                long vi2 = tri_verts[numTris * 3 + 1];
                long vi3 = tri_verts[numTris * 3 + 2];
            }
        }

        for (long m = 0; m < 3; ++m)
        {
            mesh_paths[m] = dpa[m];
            init_verts[m] = compose_pointarray(dpa[m]);
        }

        // Source mesh data init
        // connectivity info
        MFnMesh srcMesh(dpa[0]);
        MIntArray src_triCounts;
        srcMesh.getTriangles(src_triCounts, src_indeice);
        src_num_tri = 0;
        for (unsigned int f = 0; f < src_triCounts.length(); ++f)
        {
            src_num_tri += src_triCounts[f];
        }

        //Get animation settings
        auto keys = enumerate_keyframes(mesh_joint_paths);
        MTime anim_first = MAnimControl::animationStartTime();
        MTime anim_last = MAnimControl::animationEndTime();
        if (!keys.empty())
        {
            MTime anim_first = max(keys.front(), MAnimControl::animationStartTime());
            MTime anim_last = min(keys.back(), MAnimControl::animationEndTime());
        }
        const long numSamples =
            static_cast<long>((anim_last - anim_first).value() + 1);
        MAnimControl::setAnimationStartEndTime(anim_first, anim_first + numSamples - 1);
        MAnimControl::setMinMaxTime(anim_first, anim_first + numSamples - 1);
        MAnimControl::setCurrentTime(anim_first);

        //Copy the helper joints
        double transform_effect = 1.0;
        copy_helper_joints(root_joint_paths[1], root_joint_paths[2], anim_first, anim_last, numSamples, transform_effect);

        //Get vertices
        MPointArray src_default_verts = compose_pointarray(mesh_paths[0]);
        MPointArray src_deform_verts = compose_pointarray(mesh_paths[1]);
        MPointArray tar_verts = compose_pointarray(mesh_paths[2]);

        //Get the weight of vertices (including  weights of helper joints)
        vector<vector<double>> src_default_weight;
        vector<vector<double>> src_deform_weight;
        vector<vector<double>> tar_weight;
        vector<vector<double>> result_weight;
        vector<MString> src_default_joints_name;
        vector<MString> src_deform_joints_name;
        vector<MString> tar_joints_name;

        unsigned int inf_number = 32;
        compose_weight(mesh_paths[0], src_default_verts.length(), inf_number, src_default_weight, src_default_joints_name);
        compose_weight(mesh_paths[1], src_deform_verts.length(), inf_number, src_deform_weight, src_deform_joints_name);
        compose_weight(mesh_paths[2], tar_verts.length(), inf_number, tar_weight, tar_joints_name);
        compose_weight(mesh_paths[2], tar_verts.length(), inf_number, result_weight, tar_joints_name);

        vector<long> tar_joints_retarget;//tar -> src weight matching (without helper joints)
        retarget_joints(src_default_joints_name, tar_joints_name, tar_joints_retarget);
        
        //Calculate weight vector distance
        vector<vector<triangle_weight_index>> tar_best_weight_triangle_map;
        get_weight_distance(
            src_indeice, src_num_tri,
            src_default_weight, tar_weight,
            tar_joints_retarget,
            tar_best_weight_triangle_map);

        std::clock_t start = 0;
        double duration = 0;
        start = std::clock(); // get current time

        //Reflection
        rays_count = 0;
        vector<mapping_result> tar_mapping_points;
        vector<raycast_result> raycast_result_array;
        rig_reflection_geodesic_voxel(
            root_joint_paths[0], root_joint_paths[2],
            src_default_verts, tar_verts,
            src_indeice, src_num_tri,
            src_default_weight, tar_weight,
            tar_joints_name,
            tar_best_weight_triangle_map,
            sample_number,
            sample_degree,
            tar_mapping_points,
            raycast_result_array,
            debug_state);

        MAnimControl::setCurrentTime(anim_first);

        //interpolation weight
        weight_transform(
            src_default_verts,
            tar_joints_retarget,
            src_default_weight,
            src_deform_weight,
            tar_weight,
            src_indeice,
            tar_mapping_points,
            raycast_result_array,
            weight_decay,
            result_weight);

        duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        MString rays_and_duration_inf = ("rays: " + to_string(rays_count) + " time: " + to_string(duration) + " s").c_str();
        MGlobal::displayInfo(rays_and_duration_inf);
        //Assign weight
        hb_set_weight(
            mesh_paths[2],
            root_joint_paths[2],
            inf_number,
            tar_weight,
            result_weight,
            src_deform_joints_name,
            tar_joints_name,
            blend_default_weight);
    }

private:
    MDagPath mesh_paths[3];
    MDagPath root_joint_paths[3];
    MDagPathArray mesh_joint_paths;
    MIntArray tri_verts;
    MIntArray src_indeice;//triangle index of source
    long src_num_tri;//triangle numbers of source
    MPointArray init_verts[3];
};

class DeformationTransferCmd : public MPxCommand
{
public:
    virtual MStatus
        doIt(
            const MArgList& args)
    {
        //1.sample number
        //2.sample degree
        //3.weight decay
        //4.blend with default weight 
        MStatus st;
        int sample_number = args.asInt(0, &st);
        if (st == MStatus::kFailure)
        {
            sample_number = 32;
        }
        double sample_degree = args.asDouble(1, &st);
        if (st == MStatus::kFailure)
        {
            sample_degree = 5;
        }
        double weight_decay = args.asDouble(2, &st);
        if (st == MStatus::kFailure)
        {
            weight_decay = 2;
        }
        double blend_default_weight = args.asDouble(3, &st);
        if (st == MStatus::kFailure)
        {
            blend_default_weight = 1;
        }
        int debug_state = args.asInt(4, &st);
        if (st == MStatus::kFailure)
        {
            debug_state = 0;
        }
        deformation_transfer df;
        df.raycast_weight_transform(sample_number, sample_degree, weight_decay, blend_default_weight, debug_state);
        return MStatus::kSuccess;
    }
};

double
rnd()
{
    return 20.0 * rand() / (double)RAND_MAX - 10.0;
}

MStatus
initializePlugin(
    MObject obj)
{
    MStatus status;
    MFnPlugin plugin(obj, "Mukai Lab", "v.2021.12.01", "2021");
    status = plugin.registerCommand("De",
        []()->void* { return new DeformationTransferCmd; });
    CHECK_MSTATUS(status);
    return status;
}


MStatus
uninitializePlugin(
    MObject obj)
{
    MStatus status;
    MFnPlugin plugin(obj);
    status = plugin.deregisterCommand("De");
    CHECK_MSTATUS(status);
    return status;
}
