#include "mesh.h"

#include <regex>

std::vector<std::string> split(std::string s, std::string del = " ") {
    std::vector<std::string> values;
    int start, end = -1 * del.size();
    do {
        start = end + del.size();
        end = s.find(del, start);
        values.push_back(s.substr(start, end - start));
    } while (end != -1);
    return values;
}

Mesh::Mesh(utilsStructs::materialK k, double shininess,
           std::vector<Eigen::Vector4d> vertices,
           std::vector<Eigen::Vector4d> normals,
           std::vector<Eigen::Vector3d> edges,
           std::vector<Eigen::Vector4d> faces, std::shared_ptr<Sphere> cluster)
    : Object(k, shininess, utilsStructs::OBJ_TYPE::MESH),
      vertices(vertices),
      normals(normals),
      edges(edges),
      faces(faces),
      cluster(cluster) {}

Mesh::Mesh(utilsStructs::materialK k, double shininess, std::string path, std::shared_ptr<Sphere> cluster)
    : Object(k, shininess, utilsStructs::OBJ_TYPE::MESH), cluster(cluster) {
    std::ifstream objFile;
    objFile.open(path);
    std::string line;
    double id = 0;
    while (getline(objFile, line)) {
        if (line[0] == 'o') {
            std::vector<std::string> object_info = split(line);
            this->objName = object_info[1];
        }

        else if (line[0] == 'v' && line[1] == ' ') {
            std::vector<std::string> vertices_values = split(line);

            Eigen::Vector4d v(0.0, 0.0, 0.0, 1.0);
            v(0) = std::stod(vertices_values[1]);
            v(1) = std::stod(vertices_values[3]);
            v(2) = std::stod(vertices_values[2]);
            this->vertices.push_back(v);
        } else if (line[0] == 'v' && line[1] == 'n') {
            std::vector<std::string> normal_values = split(line);
            Eigen::Vector4d vn(0.0, 0.0, 0.0, 0.0);
            vn(0) = std::stod(normal_values[1]);
            vn(1) = std::stod(normal_values[3]);
            vn(2) = std::stod(normal_values[2]);

            this->normals.push_back(vn);
        } else if (line[0] == 'f') {
            std::vector<std::string> face_values = split(line);
            Eigen::Vector4d f(0.0, 0.0, 0.0, 0.0);
            double v1 = std::stod(split(face_values[1], "//")[0]);
            double v2 = std::stod(split(face_values[2], "//")[0]);
            double v3 = std::stod(split(face_values[3], "//")[0]);
            double n = std::stod(split(face_values[3], "//")[1]);

            Eigen::Vector3d e1(id++, v1, v2);
            Eigen::Vector3d e2(id++, v2, v3);
            Eigen::Vector3d e3(id++, v3, v1);

            this->edges.push_back(e1);
            this->edges.push_back(e2);
            this->edges.push_back(e3);

            f(0) = e1[0];
            f(1) = e2[0];
            f(2) = e3[0];
            f(3) = n;
            this->faces.push_back(f);
        }
    }
    objFile.close();
}

std::tuple<double, double> Mesh::intersectRay(Eigen::Vector3d O,
                                              Eigen::Vector3d D) {
    double inf = std::numeric_limits<double>::infinity();
    Eigen::Vector3d P_I(0.0, 0.0, 0.0);
    double t = inf;

    if (this->cluster != nullptr) {
        auto [tCluster1, tCluster2] = this->cluster->intersectRay(O, D);
        if (tCluster1 == inf && tCluster2 == inf) {
            return std::make_tuple(t, t);
        }
    }

    for (auto &face : this->faces) {
        double vertex_id1 = (this->edges[int(face[0])])[1] - 1;
        double vertex_id2 = (this->edges[int(face[1])])[1] - 1;
        double vertex_id3 = (this->edges[int(face[2])])[1] - 1;

        Eigen::Vector3d n = (this->normals[int(face[3]) - 1]).head<3>();
        double t_aux =
            -(O - this->vertices[int(vertex_id1)].head<3>()).dot(n) / D.dot(n);
        if (t_aux > 0.0 && t_aux < t) {
            P_I = O + t_aux * D;

            Eigen::Vector3d P1 = this->vertices[int(vertex_id1)].head<3>();

            Eigen::Vector3d P2 = this->vertices[int(vertex_id2)].head<3>();

            Eigen::Vector3d P3 = this->vertices[int(vertex_id3)].head<3>();

            Eigen::Vector3d r1 = P2 - P1;
            Eigen::Vector3d r2 = P3 - P1;
            double c1 = ((P3 - P_I).cross(P1 - P_I)).dot(n) / (r1.cross(r2)).dot(n);
            double c2 = ((P1 - P_I).cross(P2 - P_I)).dot(n) / (r1.cross(r2)).dot(n);
            double c3 = 1 - c1 - c2;
            if (c1 >= 0.0000 && c2 >= 0.0000 && c3 >= 0.0000) {
                t = t_aux;
                this->normal = n;
            }
        }
    }
    t = t - 0.0001;
    return std::make_tuple(t, t);
}

Eigen::Vector3d Mesh::getNormal(Eigen::Vector3d P_I) { return this->normal; }

void Mesh::returnToWorld(Eigen::Matrix4d cw, bool isReflection) {
    applyMatrixVertices(cw);
    applyMatrixNormals(cw);
    if (!isReflection) {
        Eigen::Matrix4d m = matrix::translate(-this->x, -this->y, -this->z);
        if (this->cluster != nullptr) {
            this->cluster->returnToWorld(cw, false);
        }
        applyMatrixVertices(m);
        applyMatrixNormals(m);
    } else {
        // this->coordinatesAux = Eigen::Vector4d(this->x, this->y, this->z, 1.0);
        // this->coordinatesAux = cw * this->coordinatesAux;
    }
    return;
}

void Mesh::backToCamera(Eigen::Matrix4d wc) {
    applyMatrixVertices(wc);
    applyMatrixNormals(wc);
    // this->coordinatesAux = wc * this->coordinatesAux;
    return;
}

void Mesh::applyMatrixVertices(Eigen::Matrix4d m) {
    for (Eigen::Vector4d &v : vertices) {
        v(3) = 1.0;
        v = m * v;
    }
    return;
}

void Mesh::applyMatrixNormals(Eigen::Matrix4d m) {
    for (Eigen::Vector4d &n : normals) {
        n = (m * n).normalized();
    }
    return;
}

void Mesh::scale(double x, double y, double z) {
    Eigen::Matrix4d m = matrix::scale(x, y, z);
    if (this->cluster != nullptr) {
        double maxDimension = std::max(std::max(x, y), z);
        cluster->scale(maxDimension);
    }
    applyMatrixVertices(m);
    applyMatrixNormals((m.transpose()).inverse());

    return;
}
void Mesh::shear(double delta, matrix::SHEAR_AXIS axis) {
    Eigen::Matrix4d m = matrix::shear(delta, axis);
    applyMatrixVertices(m);
    applyMatrixNormals((m.transpose()).inverse());
    return;
}
void Mesh::translate(double x, double y, double z, Eigen::Matrix4d wc) {
    this->x = x;
    this->y = y;
    this->z = z;
    // this->coordinatesAux = Eigen::Vector4d(this->x, this->y, this->z, 1.0);
    Eigen::Matrix4d m = matrix::translate(x, y, z);
    if (this->cluster != nullptr) {
        this->cluster->translate(x, y, z, wc);
    }
    applyMatrixVertices(m);
    applyMatrixNormals(m);
    applyMatrixVertices(wc);
    applyMatrixNormals(wc);
    return;
}
void Mesh::rotate(double theta, matrix::AXIS axis) {
    Eigen::Matrix4d m = matrix::rotate(theta, axis);
    applyMatrixVertices(m);
    applyMatrixNormals(m);
    return;
}
void Mesh::reflection(matrix::REFLECTION_AXIS axis, std::vector<std::shared_ptr<Object>> &objects, Eigen::Matrix4d wc) {
    Eigen::Matrix4d m = matrix::reflection(axis);
    Mesh reflectedMesh(this->K, this->m, this->vertices, this->normals, this->edges, this->faces);

    // Updating object center
    // reflectedMesh.coordinatesAux = this->coordinatesAux;
    // reflectedMesh.coordinatesAux = m * reflectedMesh.coordinatesAux;
    // reflectedMesh.coordinatesAux = wc * reflectedMesh.coordinatesAux;

    // Operating vertices and normals
    reflectedMesh.applyMatrixVertices(m);
    reflectedMesh.applyMatrixNormals(m);

    this->coordinatesAux(0) = this->x;
    this->coordinatesAux(1) = this->y;
    this->coordinatesAux(2) = this->z;
    this->coordinatesAux = m * this->coordinatesAux;
    std::cout << this->coordinatesAux(0) << " " << this->coordinatesAux(1) << " " << this->coordinatesAux(2) << "\n";

    reflectedMesh.x = this->coordinatesAux(0);
    reflectedMesh.y = this->coordinatesAux(1);
    reflectedMesh.z = this->coordinatesAux(2);

    // reflectedMesh.x = reflectedMesh.coordinatesAux(0);
    // reflectedMesh.y = reflectedMesh.coordinatesAux(1);
    // reflectedMesh.z = reflectedMesh.coordinatesAux(2);

    reflectedMesh.backToCamera(wc);

    objects.push_back(std::make_shared<Mesh>(reflectedMesh));
    return;
}
