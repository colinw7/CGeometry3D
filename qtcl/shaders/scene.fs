#version 330 core

in vec3 FragPos;
in vec3 Normal;
in vec3 Color;
in vec2 TexCoords;

out vec4 FragColor;

uniform vec3 viewPos;

uniform vec3  ambientColor;
uniform float ambientStrength;
uniform float diffuseStrength;
uniform vec3  emissionColor;
uniform float emissiveStrength;
uniform vec3  specularColor;
uniform float specularStrength;
uniform float shininess;
uniform bool  fixedDiffuse;

//--- Textures

struct TextureData {
  bool      enabled;
  sampler2D texture;
};

uniform TextureData diffuseTexture;
uniform TextureData normalTexture;
uniform TextureData specularTexture;
uniform TextureData emissiveTexture;

//--- State

uniform bool isSelected;
uniform bool isWireframe;
uniform bool isLine;

uniform float transparency;

uniform vec3  selectColor;
uniform vec3  wireframeColor;
uniform float wireframeTransparency;

//---

vec3 calcNormal() {
  if (normalTexture.enabled) {
    vec3 norm = texture(normalTexture.texture, TexCoords).rgb;
    norm = normalize(norm*2.0 - 1.0); // this normal is in tangent space
    return norm;
  }
  else
    return normalize(Normal);
}

/*
float calcDiffuseFactor(vec3 lightDir, vec3 nrm) {
  float diffAmt = max(dot(nrm, lightDir), 0.0);
  return diffAmt;
}
*/

vec3 calcDiffuseColor() {
  if (diffuseTexture.enabled) {
    vec4 textureColor = texture(diffuseTexture.texture, TexCoords);

    if (textureColor.a < 0.1)
      discard;

    return textureColor.rgb;
  }
  else
    return diffuseStrength*Color;
}

/*
float calcSpecularFactor(vec3 lightDir, vec3 viewDir, vec3 nrm, float shininess) {
  vec3 reflectDir = reflect(-lightDir, nrm);
  float specAmt = max(dot(viewDir, reflectDir), 0.0);
  return pow(specAmt, shininess);
}
*/

vec3 calcSpecularColor() {
  if (specularTexture.enabled)
    return texture(specularTexture.texture, TexCoords).rgb;
  else
    return specularColor*specularStrength;
}

vec3 calcEmissionColor() {
  if (emissiveTexture.enabled)
    return texture(emissiveTexture.texture, TexCoords).rgb;
  else
    return emissiveStrength*emissionColor;
}

//---

void main() {
  // normal
  vec3 norm = calcNormal();

  // ambient
  vec3 ambient = ambientStrength*ambientColor;

  vec3 result = ambient;

  // diffuse color
  vec3 diffuseColor = calcDiffuseColor();

  // specular color
  vec3 specColor = calcSpecularColor();

  vec3 viewDir = normalize(viewPos - FragPos);

  // baked diffuse lighting
  float diffFactor = 1.0;

  if (! fixedDiffuse)
    diffFactor = max(dot(norm, viewDir), 0.0);

  result += diffFactor*diffuseColor;

  // add emission
  vec3 emissionColor = calcEmissionColor();

  result += emissionColor;

  // adjust color by state

  if (! isWireframe) {
    if (! isSelected) {
      FragColor = vec4(result, transparency);
    } else {
      vec3 selectColor1 = mix(result, selectColor, 0.4);

      FragColor = vec4(selectColor1, 1.0);
    }
  } else {
    if (! isSelected) {
      if (! isLine) {
        FragColor = vec4(wireframeColor, wireframeTransparency);
      } else {
        FragColor = vec4(result, transparency);
      }
    } else {      
      FragColor = vec4(selectColor, 1.0);
    }
  }
}
