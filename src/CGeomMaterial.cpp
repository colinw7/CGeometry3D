#include <CGeomMaterial.h>

CGeomMaterialMgr::
CGeomMaterialMgr()
{
}

void
CGeomMaterialMgr::
addMaterial(CGeomMaterial *material)
{
  material->setId(++ind_);

  materialMap_[material->name()] = material;
}

CGeomMaterial *
CGeomMaterialMgr::
getMaterial(const std::string &name) const
{
  auto pm = materialMap_.find(name);
  if (pm == materialMap_.end())
    return nullptr;

  return (*pm).second;
}

std::vector<CGeomMaterial *>
CGeomMaterialMgr::
getMaterials() const
{
  std::vector<CGeomMaterial *> materials;

  for (const auto &pm : materialMap_)
    materials.push_back(pm.second);

  return materials;
}

CGeomMaterial *
CGeomMaterialMgr::
getMaterialById(uint id) const
{
  for (const auto &pm : materialMap_) {
    if (pm.second->id() == id)
      return pm.second;
  }

  return nullptr;
}

//------

/* static */
const char *
CGeomMaterial::
shadingName(const Shading &shading)
{
  switch (shading) {
    case Shading::WIRE   : return "Wire";
    case Shading::FLAT   : return "Flat";
    case Shading::GOURAUD: return "Gouraud";
    case Shading::PHONG  : return "Phong";
    case Shading::METAL  : return "Metal";
    case Shading::BLINN  : return "BLinn";
    default              : return "None";
  }
}

CGeomMaterial::
CGeomMaterial()
{
}

CGeomMaterial::
CGeomMaterial(const CMaterial &material) :
 material_(material)
{
}
