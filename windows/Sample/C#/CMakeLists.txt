
message("***************build C#")
project(demoCSharp LANGUAGES CSharp)

set(LX_LIBRARY_NAME LxCameraApi)
ADD_LIBRARY(${LX_LIBRARY_NAME}Net  SHARED  ${LX_LIBRARY_NAME}.cs)

ADD_EXECUTABLE(demoCSharp demo.cs)    
target_link_libraries(demoCSharp    ${LX_LIBRARY_NAME}Net)     
set_property(TARGET ${LX_LIBRARY_NAME}Net demoCSharp PROPERTY VS_DOTNET_REFERENCES
		"System"
		"System.Drawing" 
		"Microsoft.CSharp" 
		"WindowsBase" 
		)

