/*
Open Asset Import Library (assimp)
----------------------------------------------------------------------

Copyright (c) 2006-2022, assimp team

All rights reserved.

Redistribution and use of this software in source and binary forms,
with or without modification, are permitted provided that the
following conditions are met:

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

----------------------------------------------------------------------
*/

/** @file  BMDImporter.h
 *  @brief Definition of the Valve BMD file format
 */
#ifndef BMD_FILE_IMPORTER_H_INC
#define BMD_FILE_IMPORTER_H_INC

#include <assimp/BaseImporter.h>
#include <assimp/ParsingUtils.h>
#include <assimp/types.h>

struct aiNode;
struct aiMesh;
struct aiMaterial;

namespace Assimp {

// ---------------------------------------------------------------------------
/** Used to load Half-life 1 and 2 SMD models
*/
class ASSIMP_API BmdImporter : public BaseImporter {
public:
    BmdImporter() = default;
    ~BmdImporter() override = default;

    // -------------------------------------------------------------------
    /** Returns whether the class can handle the format of the given file.
     * See BaseImporter::CanRead() for details.
     */
    bool CanRead(const std::string &pFile, IOSystem *pIOHandler, bool checkSig) const override;

protected:
    // -------------------------------------------------------------------
    /** Return importer meta information.
     * See #BaseImporter::GetInfo for the details
     */
    const aiImporterDesc* GetInfo () const override;

    // -------------------------------------------------------------------
    /** Imports the given file into the given scene structure.
    * See BaseImporter::InternReadFile() for details
    */
    void InternReadFile( const std::string& pFile, aiScene* pScene,
        IOSystem* pIOHandler) override;

private:

    bool ReadA3DLitModel(std::istream &stream, aiScene *scene) const;
    bool ReadA3DLitMeshShareData(std::istream &stream, aiScene *scene) const;
    bool ReadA3DLitMesh(std::istream &stream, aiScene *scene) const;
    static aiString read_string(std::istream &stream);

    static void seek_string(std::istream &stream);

    template <class T>
    static T read_element(std::istream &stream) {
        T element{};
        stream.read(reinterpret_cast<char *>(&element), sizeof element);
        return element;
    }

    template <class T>
    static void seek_element(std::istream &stream) {
        stream.seekg(sizeof T, std::ios::cur);
    }

    static void unpack_pos_float(float &dst, const char *src);

    static void unpack_pos(aiVector3D &v, const char *src);

    static void unpack_norm_float(float &dst, const char src);

    static void unpack_norm(aiVector3D &v_norm, const char *src);
};

} // end of namespace Assimp

#endif // BMD_FILE_IMPORTER_H_INC
