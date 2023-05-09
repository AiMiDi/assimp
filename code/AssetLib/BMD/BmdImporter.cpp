/*
---------------------------------------------------------------------------
Open Asset Import Library (assimp)
---------------------------------------------------------------------------

Copyright (c) 2006-2022, assimp team

All rights reserved.

Redistribution and use of this software in source and binary forms,
with or without modification, are permitted provided that the following
conditions are met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.

* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other
  materials provided with the distribution.

* Neither the name of the assimp team, nor the names of its
  contributors may be used to endorse or promote products
  derived from this software without specific prior
  written permission of the assimp team.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
---------------------------------------------------------------------------
*/

/** @file  BMDImporter.cpp
 *  @brief Implementation of the BMD importer class
 */

#ifndef ASSIMP_BUILD_NO_BMD_IMPORTER

#include <assimp/fast_atof.h>
#include <assimp/importerdesc.h>
#include <assimp/scene.h>
#include <memory>
#include <assimp/IOSystem.hpp>

// internal headers
#include "BmdImporter.h"

#ifndef _MSC_VER
#define strtok_s strtok_r
#endif

using namespace Assimp;

typedef uint32_t A3DCOLOR;

struct A3DCOLORVALUE {
    float r, g, b, a;
};

struct A3DVECTOR3 {
    float x, y, z;
};

struct A3DLVERTEX {
    float x, y, z;
    uint32_t diffuse;
    uint32_t specular;
    float tu, tv;
};

struct A3DLMVERTEX_WITHOUTNORMAL {
    A3DVECTOR3 pos;
    A3DCOLOR diffuse;

    float u;
    float v;
};

#pragma pack(1)
struct A3DLMVERTEX_OPTIMISZE {
    char pos[9];
    float u;
    float v;
};
#pragma pack()

struct A3DAABB {
    A3DVECTOR3 Center;
    A3DVECTOR3 Extents;
    A3DVECTOR3 Mins;
    A3DVECTOR3 Maxs;
};


static constexpr aiImporterDesc desc =
{
    "BMD Importer",
    "",
    "",
    "",
    aiImporterFlags_SupportTextFlavour,
    0,
    0,
    0,
    0,
    "bmd"
};

// ------------------------------------------------------------------------------------------------
// Returns whether the class can handle the format of the given file.
bool BmdImporter::CanRead(const std::string &filename, IOSystem* pIOHandler, bool) const {
    static const char *tokens[] = { "MOXB" };
    return SimpleExtensionCheck(filename, "bmd") &&
    SearchFileHeaderForToken(pIOHandler, filename, tokens, std::size(tokens));
}

// ------------------------------------------------------------------------------------------------
// Get a list of all supported file extensions
const aiImporterDesc *BmdImporter::GetInfo() const {
    return &desc;
}

// ------------------------------------------------------------------------------------------------
// Imports the given file into the given scene structure.
void BmdImporter::InternReadFile(const std::string &pFile, aiScene *pScene, IOSystem *pIOHandler)
{
    auto streamCloser = [&](IOStream *pStream) {
        pIOHandler->Close(pStream);
    };

    const std::unique_ptr<IOStream, decltype(streamCloser)> file_stream(pIOHandler->Open(pFile, " rb"), streamCloser);

    if (file_stream == nullptr) {
        throw DeadlyImportError("Failed to open file ", pFile, ".");
    }

    std::vector<char> contents(file_stream->FileSize());
    file_stream->Read(contents.data(), 1, contents.size());

    std::istringstream iss(std::string(contents.begin(), contents.end()));

    // Generate the root-node
    pScene->mRootNode = new aiNode("BmdRoot");

    ReadA3DLitModel(iss, pScene);
}

bool BmdImporter::ReadA3DLitModel(std::istream &stream, aiScene *scene) const {
    // first of all, read the file head
    char magic[4];
    stream.read(magic, std::size(magic));

    const auto dw_version = read_element<uint32_t>(stream);

    if (dw_version >= 0x10000003) {
        const auto local_save_share_data = read_element<bool>(stream);

        // ShareDataPath
        seek_string(stream);

        if (local_save_share_data) {
            if (!ReadA3DLitMeshShareData(stream, scene))
                return false;
        }
        
        const auto scale = read_element<aiVector3D>(stream);
        const auto dir = read_element<aiVector3D>(stream);
        const auto up = read_element<aiVector3D>(stream);
        const auto pos = read_element<aiVector3D>(stream);

        // TODO: rotation
        scene->mRootNode->mTransformation = aiMatrix4x4{ scale, {}, pos };

        if (!ReadA3DLitMesh(stream, scene))
            return false;

    } else if (dw_version < 0x10000003) {
        const auto scale = read_element<aiVector3D>(stream);
        const auto dir = read_element<aiVector3D>(stream);
        const auto up = read_element<aiVector3D>(stream);
        const auto pos = read_element<aiVector3D>(stream);

        // TODO: rotation
        scene->mRootNode->mTransformation = aiMatrix4x4{ scale, {}, pos };

        if (!ReadA3DLitMesh(stream, scene))
            return false;
    } else {
        return false;
    }

    return true;
}

bool BmdImporter::ReadA3DLitMeshShareData(std::istream &stream, aiScene *scene) const {
    // dwVersion
    seek_element<uint32_t>(stream);

    const auto num_meshes = read_element<int>(stream);
    scene->mNumMeshes = num_meshes;

    scene->mMeshes = new aiMesh *[num_meshes]();
    scene->mMaterials = new aiMaterial *[num_meshes]();
    scene->mRootNode->mNumChildren = num_meshes;
    scene->mRootNode->mChildren = new aiNode *[num_meshes]();

    for (int mesh_index = 0; mesh_index < num_meshes; mesh_index++) {
        auto *mesh = scene->mMeshes[mesh_index];
        auto *mat = scene->mMaterials[mesh_index];
        auto *node = scene->mRootNode->mChildren[mesh_index];
        node->mParent = scene->mRootNode;
        const auto dw_version = read_element<uint32_t>(stream);

        const auto is_optimize = read_element<bool>(stream);

        const auto texture_path = read_string(stream);
        mat->AddProperty(&texture_path, AI_MATKEY_TEXTURE(aiTextureType_DIFFUSE, 0));

        // TexDXT1AlphaFlag
        stream.seekg(sizeof(int), std::ios::cur);

        const auto vertex_count = read_element<int>(stream);
        const auto face_count = read_element<int>(stream);

        if (is_optimize) {
            const auto vertices = std::make_unique<A3DLMVERTEX_OPTIMISZE[]>(vertex_count);
            stream.read(reinterpret_cast<char *>(vertices.get()), sizeof A3DLMVERTEX_OPTIMISZE * vertex_count);

            const auto normal = std::make_unique<char[]>(vertex_count * 3ULL);
            stream.read(normal.get(), vertex_count * 3LL);

            const auto indices = std::make_unique<uint16_t[]>(face_count * 3ULL);
            stream.read(reinterpret_cast<char *>(indices.get()), sizeof uint16_t * face_count * 3ULL);

            mesh->mNumFaces = static_cast<unsigned int>(face_count);
            mesh->mFaces = new aiFace[mesh->mNumFaces]();
            for (int face_index = 0, vertex_index = 0; face_index < face_count; face_index++) {
                auto &face = mesh->mFaces[face_index];
                face.mNumIndices = 3;
                face.mIndices = new unsigned[3];
                for (unsigned face_vertex_index = 0; face_vertex_index < 3; ++face_vertex_index, ++vertex_index) {
                    face.mIndices[face_vertex_index] = indices[vertex_index];
                }
            }

            mesh->mNumVertices = vertex_count;
            mesh->mVertices = new aiVector3D[mesh->mNumVertices];
            mesh->mTextureCoords[0] = new aiVector3D[mesh->mNumVertices];
            for (int vertex_index = 0; vertex_index < vertex_count; vertex_index++) {
                const auto &[pos, u, v] = vertices[vertex_index];

                auto &vertex = mesh->mVertices[vertex_index];
                unpack_pos(vertex, pos);

                auto &uv = mesh->mTextureCoords[0][vertex_index];
                uv.x = u;
                uv.y = v;

                const char *cur_normal = normal.get() + static_cast<ptrdiff_t>(vertex_index * 3);
                unpack_norm(mesh->mNormals[vertex_index], cur_normal);
            }
        } else {
            const auto vertices = std::make_unique<A3DLMVERTEX_WITHOUTNORMAL[]>(vertex_count);
            stream.read(reinterpret_cast<char *>(vertices.get()), sizeof A3DLMVERTEX_WITHOUTNORMAL * vertex_count);

            const auto normals = new aiVector3D[vertex_count];
            stream.read(reinterpret_cast<char *>(normals), sizeof aiVector3D * vertex_count);

            const auto indices = std::make_unique<uint16_t[]>(face_count * 3ULL);
            stream.read(reinterpret_cast<char *>(indices.get()), sizeof uint16_t * face_count * 3ULL);

            mesh->mNumFaces = static_cast<unsigned int>(face_count);
            mesh->mFaces = new aiFace[mesh->mNumFaces]();
            for (int face_index = 0, vertex_index = 0; face_index < face_count; face_index++) {
                auto &face = mesh->mFaces[face_index];
                face.mNumIndices = 3;
                face.mIndices = new unsigned int[3];
                for (unsigned int face_vertex_index = 0; face_vertex_index < 3; ++face_vertex_index, ++vertex_index) {
                    face.mIndices[face_vertex_index] = indices[vertex_index];
                }
            }

            mesh->mNumVertices = vertex_count;
            mesh->mVertices = new aiVector3D[mesh->mNumVertices];
            mesh->mTextureCoords[0] = new aiVector3D[mesh->mNumVertices];
            for (int vertex_index = 0; vertex_index < vertex_count; vertex_index++) {
                auto &vertex = mesh->mVertices[vertex_index];
                auto &uv = mesh->mTextureCoords[0][vertex_index];
                vertex.x = vertices[vertex_index].pos.x;
                vertex.y = vertices[vertex_index].pos.y;
                vertex.z = vertices[vertex_index].pos.z;
                uv.x = vertices[vertex_index].u;
                uv.y = vertices[vertex_index].v;
            }

            mesh->mNormals = normals;
        }

        if (dw_version >= 0x1000000c) {
            // Tangent
            stream.seekg(sizeof A3DCOLOR * vertex_count, std::ios::cur);
        }

        A3DAABB AABB{};
        stream.read(reinterpret_cast<char *>(&AABB), sizeof A3DAABB);
        mesh->mAABB.mMax.x = AABB.Maxs.x;
        mesh->mAABB.mMax.y = AABB.Maxs.y;
        mesh->mAABB.mMax.z = AABB.Maxs.z;
        mesh->mAABB.mMin.x = AABB.Mins.x;
        mesh->mAABB.mMin.y = AABB.Mins.y;
        mesh->mAABB.mMin.z = AABB.Mins.z;
    }

    return true;
}

bool BmdImporter::ReadA3DLitMesh(std::istream &stream, aiScene *scene) const {
    const auto num_meshes = read_element<int>(stream);
    scene->mNumMeshes = num_meshes;
    scene->mMeshes = new aiMesh *[num_meshes]();
    scene->mNumMaterials = num_meshes;
    scene->mMaterials = new aiMaterial *[num_meshes]();
    scene->mRootNode->mNumChildren = num_meshes;
    scene->mRootNode->mChildren = new aiNode *[num_meshes]();
    for (int mesh_index = 0; mesh_index < num_meshes; mesh_index++) {
        auto *&mesh = scene->mMeshes[mesh_index];
        mesh = new aiMesh();
        auto *&mat = scene->mMaterials[mesh_index];
        mat = new aiMaterial();
        auto *&node = scene->mRootNode->mChildren[mesh_index];
        node = new aiNode();
        node->mParent = scene->mRootNode;
        // set mesh to node
        node->mNumMeshes = 1;
        node->mMeshes = new unsigned[1]();
        node->mMeshes[0] = mesh_index;
        // set material to mesh
        mesh->mMaterialIndex = mesh_index;

        auto const dw_version = read_element<uint32_t>(stream);

        if (dw_version == 0x10000001) {
            return false;
        }

        // b,a,9
        if (dw_version >= 0x1000000b) {
            // is_optimize
            seek_element<bool>(stream);
            // name
            seek_string(stream);
            const auto color_count = read_element<int>(stream);

            // DayColors
            stream.seekg(sizeof A3DCOLOR * color_count, std::ios::cur);

            if (dw_version >= 0x1000000d) {
                // NightColors
                stream.seekg(sizeof A3DCOLOR * color_count, std::ios::cur);
            }
        } else if (dw_version >= 0x1000000a) {
            const auto is_optimize = read_element<bool>(stream);
            // name
            seek_string(stream);
            const auto color_count = read_element<int>(stream);
            const auto has_extra_colors = read_element<bool>(stream);

            if (is_optimize) {
                // DayColors
                stream.seekg(color_count * 3LL, std::ios::cur);

                if (has_extra_colors)
                    stream.seekg(color_count * 3LL, std::ios::cur);
            }else {
                // DayColors
                stream.seekg(sizeof A3DCOLOR * color_count, std::ios::cur);

                if (has_extra_colors)
                    stream.seekg(sizeof A3DCOLOR * color_count, std::ios::cur);
            }
        } /*TODO: else if (dw_version >= 0x10000009) {
            const auto is_optimize = read_element<bool>(stream);
            // name
            seek_string(stream);
        }*/

        // common

        char sz_name[64];
        stream.read(sz_name, std::size(sz_name));
        node->mName = sz_name;

        char sz_texture_map[256];
        stream.read(sz_texture_map, std::size(sz_texture_map));
        aiString texture_path(sz_texture_map);
        mat->AddProperty(&texture_path, AI_MATKEY_TEXTURE(aiTextureType_DIFFUSE, 0));

        if (dw_version >= 0x10000007) {
            // TexDXT1AlphaFlag
            seek_element<int>(stream);
        }

        const auto vertex_count = read_element<int>(stream);
        mesh->mNumVertices = vertex_count;
        mesh->mVertices = new aiVector3D[mesh->mNumVertices]();
        mesh->mTextureCoords[0] = new aiVector3D[mesh->mNumVertices]();

        const auto face_count = read_element<int>(stream);
        mesh->mNumFaces = static_cast<unsigned int>(face_count);
        mesh->mFaces = new aiFace[mesh->mNumFaces]();

        bool has_extra_colors = false;
        if (dw_version >= 0x10000006) {
            has_extra_colors = read_element<bool>(stream);
        }

        if (dw_version == 0x10000002) {
            const auto vertices = std::make_unique<A3DLVERTEX[]>(vertex_count);
            stream.read(reinterpret_cast<char *>(vertices.get()), sizeof A3DLVERTEX * vertex_count);

            for (int vertex_index = 0; vertex_index < vertex_count; vertex_index++) {
                auto &vertex = mesh->mVertices[vertex_index];
                auto &uv = mesh->mTextureCoords[0][vertex_index];
                vertex.x = vertices[vertex_index].x;
                vertex.y = vertices[vertex_index].y;
                vertex.z = vertices[vertex_index].z;
                uv.x = vertices[vertex_index].tu;
                uv.y = vertices[vertex_index].tv;
            }
        } else if (dw_version == 0x10000003) {
            const auto vertices = std::make_unique<A3DLVERTEX[]>(vertex_count);
            stream.read(reinterpret_cast<char *>(vertices.get()), sizeof A3DLVERTEX * vertex_count);

            for (int vertex_index = 0; vertex_index < vertex_count; vertex_index++) {
                auto &vertex = mesh->mVertices[vertex_index];
                auto &uv = mesh->mTextureCoords[0][vertex_index];
                vertex.x = vertices[vertex_index].x;
                vertex.y = vertices[vertex_index].y;
                vertex.z = vertices[vertex_index].z;
                uv.x = vertices[vertex_index].tu;
                uv.y = vertices[vertex_index].tv;
            }
        } else if (dw_version >= 0x10000004) {
            const auto vertices = std::make_unique<A3DLMVERTEX_WITHOUTNORMAL[]>(vertex_count);
            stream.read(reinterpret_cast<char *>(vertices.get()), sizeof A3DLMVERTEX_WITHOUTNORMAL * vertex_count);

            for (int vertex_index = 0; vertex_index < vertex_count; vertex_index++) {
                auto &vertex = mesh->mVertices[vertex_index];
                auto &uv = mesh->mTextureCoords[0][vertex_index];
                vertex.x = vertices[vertex_index].pos.x;
                vertex.y = vertices[vertex_index].pos.y;
                vertex.z = vertices[vertex_index].pos.z;
                uv.x = vertices[vertex_index].u;
                uv.y = vertices[vertex_index].v;
            }
        }

        // common

        const auto indices = std::make_unique<uint16_t[]>(face_count * 3ULL);
        stream.read(reinterpret_cast<char *>(indices.get()), sizeof uint16_t * face_count * 3ULL);

        for (int face_index = 0, vertex_index = 0; face_index < face_count; face_index++) {
            auto &face = mesh->mFaces[face_index];
            face.mNumIndices = 3;
            face.mIndices = new unsigned int[3]();
            for (unsigned int face_vertex_index = 0; face_vertex_index < 3; ++face_vertex_index, ++vertex_index) {
                face.mIndices[face_vertex_index] = indices[vertex_index];
            }
        }

        const auto normals = new aiVector3D[vertex_count]();
        stream.read(reinterpret_cast<char *>(normals), sizeof aiVector3D * vertex_count);

        mesh->mNormals = normals;

        // Colors
        if (dw_version >= 0x10000003 && dw_version < 0x10000006) {
            // DayColors & NightColors
            stream.seekg(sizeof A3DCOLOR * vertex_count * 2, std::ios::cur);
        } else if (dw_version >= 0x10000006) {

            if (dw_version < 0x10000008)
                stream.seekg(sizeof A3DCOLOR * vertex_count, std::ios::cur);

            if (has_extra_colors) {
                stream.seekg(sizeof A3DCOLOR * vertex_count, std::ios::cur);

                if (dw_version < 0x10000008)
                    stream.seekg(sizeof A3DCOLOR * vertex_count, std::ios::cur);
            }
        }

        A3DAABB AABB{};
        stream.read(reinterpret_cast<char *>(&AABB), sizeof A3DAABB);
        mesh->mAABB.mMax.x = AABB.Maxs.x;
        mesh->mAABB.mMax.y = AABB.Maxs.y;
        mesh->mAABB.mMax.z = AABB.Maxs.z;
        mesh->mAABB.mMin.x = AABB.Mins.x;
        mesh->mAABB.mMin.y = AABB.Mins.y;
        mesh->mAABB.mMin.z = AABB.Mins.z;

        if (dw_version >= 0x10000005) {
            // A3DMaterial

            char sz_line_buffer[2048]{0};
            stream.read(sz_line_buffer, std::size(sz_line_buffer));
            aiString mat_name("mat_");
            mat_name.Append(sz_line_buffer);
            mat->AddProperty(&mat_name, AI_MATKEY_NAME);
            
            const auto ambient_4 = read_element<A3DCOLORVALUE>(stream);
            const auto diffuse_4 = read_element<A3DCOLORVALUE>(stream);
            const auto emissive_4 = read_element<A3DCOLORVALUE>(stream);
            const auto specular_4 = read_element<A3DCOLORVALUE>(stream);

            const aiColor3D ambient{ ambient_4.r, ambient_4.g, ambient_4.b };
            const aiColor3D diffuse{ diffuse_4.r, diffuse_4.g, diffuse_4.b };
            const aiColor3D emissive{ emissive_4.r, emissive_4.g, emissive_4.b };
            const aiColor3D specular{ specular_4.r, specular_4.g, specular_4.b };

            mat->AddProperty(&diffuse, 1, AI_MATKEY_COLOR_DIFFUSE);
            mat->AddProperty(&specular, 1, AI_MATKEY_COLOR_SPECULAR);
            mat->AddProperty(&ambient, 1, AI_MATKEY_COLOR_AMBIENT);
            mat->AddProperty(&emissive, 1, AI_MATKEY_COLOR_EMISSIVE);
            mat->AddProperty(&diffuse_4.a, 1, AI_MATKEY_OPACITY);

            const auto specular_power = read_element<float>(stream);
            mat->AddProperty(&specular_power, 1, AI_MATKEY_SPECULAR_FACTOR);

            // is_2sided
            seek_element<bool>(stream);
        }
    }

    return false;
}

aiString BmdImporter::read_string(std::istream &stream) {
    aiString str;

    int str_len = 0;
    stream.read(reinterpret_cast<char *>(&str_len), sizeof str_len);

    if (str_len > 0 && str_len <= 1000) {
        stream.read(str.data, str_len);
    }

    return str;
}

void BmdImporter::seek_string(std::istream &stream) {
    int str_len = 0;
    stream.read(reinterpret_cast<char *>(&str_len), sizeof str_len);
    stream.seekg(str_len, std::ios::cur);
}

void BmdImporter::unpack_pos_float(float &dst, const char *src) {
    char n[4];
    n[0] = src[0];
    n[1] = src[1];
    n[2] = src[2];
    n[3] = static_cast<char>(src[2] >> 7);
    dst = static_cast<float>(*reinterpret_cast<int *>(n)) / static_cast<float>(1 << 12);
}

void BmdImporter::unpack_pos(aiVector3D &v, const char *src) {
    unpack_pos_float(v.x, src);
    unpack_pos_float(v.y, src + 3);
    unpack_pos_float(v.z, src + 6);
}

void BmdImporter::unpack_norm_float(float &dst, const char src) {
    dst = static_cast<float>(src) / 127.f;
}

void BmdImporter::unpack_norm(aiVector3D &v_norm, const char *src) {
    unpack_norm_float(v_norm.x, src[0]);
    unpack_norm_float(v_norm.y, src[1]);
    unpack_norm_float(v_norm.z, src[2]);
    v_norm.Normalize();
}

#endif 
