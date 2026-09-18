message("***************build C#")
project(demoCSharp LANGUAGES CSharp)

set(LX_LIBRARY_NAME LxCameraApi)

ADD_EXECUTABLE(demoCSharp demo_camera.cs LxCameraApi.cs)
target_link_libraries(demoCSharp ${LX_LIBRARY_NAME})
set_property(TARGET ${LX_LIBRARY_NAME} demoCSharp PROPERTY VS_DOTNET_REFERENCES
		"System"
		"System.Drawing" 
		"Microsoft.CSharp" 
		"WindowsBase" 
		)
