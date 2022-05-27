import os, sys
import subprocess
import ruamel.yaml

# Script which dynamically searches for the packages that exist in the workspace and that have documentation (and are not excluded from the search)
script_name = 'docs/scripts/get_packages_with_docs.sh'

# Name of the file that will include the name of the packages to include in the documentation
config_file = 'mkdocs.yml'

# Name of the key inside the main mkdocs.yml file where the documentation for each package will be placed
nav_key = 'Packages documentation'

# File with packages to ignore when generating doxygen
ignore_doxygen = 'docs/ignore_packages_doxygen.txt'

# Directory to save the output of the doxygen
output_doxygen_directory = os.getcwd() + '/docs/api/packages'

# Setup the location for the Doxyfile template and specifity the output location for the generated files
doxyfile_location = os.getcwd() + '/docs/scripts/Doxyfile'
output_location = os.getcwd() + '/docs/xml_doxygen'

# --------------------------------------------------
# Search for packages with docs and mkdocs.yml files
# --------------------------------------------------

# Run the bash script that outputs the directory of the packages with documentation (that are not excluded in the ignore_packages file)
result = subprocess.run(['bash', script_name], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

# If there was an error getting the list of packages that contain documentation, then stop here
if result.stderr != '':
    raise Exception('Could not discover packages that contain documentation: ' + str(result.stderr))

# Get the path to the packages that contain documentation (and were not excluded)
packages_with_docs = result.stdout.split('\n')
packages_with_docs.pop(-1)

# Pre-process the path so that our data stays inside each package correctly
packages = []
for package_path in packages_with_docs:
    
    # Split the path of the package
    path_by_sub_strings = package_path.split('/')

    # Remove the underbars of the name of the packages (make the documentation look prettier)
    for i in range(len(path_by_sub_strings)):
        path_by_sub_strings[i] = path_by_sub_strings[i].replace("_", " ").title()

    # Remove the empty string that is generated in the first item of the list
    path_by_sub_strings.pop(0)

    print('Documentation detected in: ' + package_path)
    
    # Add the packages according to a sub-folder structure to the packages list
    current_pkg = packages
    for i in range(len(path_by_sub_strings)):
        
        # If we are at in the sub-folders (and not yet in the package name), keep adding the tree to the dictionary
        if i != len(path_by_sub_strings)-1:

            # Check if there already exists a meta-package with the same name in the sub-list of packages
            found_meta_package_with_same_name = False
            for meta_package in current_pkg:
                
                if path_by_sub_strings[i] in meta_package:
                    current_pkg = meta_package[path_by_sub_strings[i]]
                    found_meta_package_with_same_name = True
                    break
            
            # If there was not meta package with the same name in the list, then just create one
            if not found_meta_package_with_same_name:
                current_pkg.append({path_by_sub_strings[i]: []})
                current_pkg = current_pkg[-1][path_by_sub_strings[i]]
        
        # If we have reached the package name, make the value corresponding to the key to the path of the documentation yaml file
        else:
            current_pkg.append({path_by_sub_strings[i]: '!include .' + package_path + '/mkdocs.yml'})

# ----------------------------------------------------------------
# Generate the doxygen for the packages with documentation enabled
# ----------------------------------------------------------------

# Read the packages to ignore when generating the doxygen
doxy_ignore_packages = []
with open(ignore_doxygen, 'r') as fp:
    doxy_ignore_packages = fp.read().split('\n')

# Generate doxygen for each package with documentation that we do not wish to ignore
for package in packages_with_docs:

    skip_package = False

    # Check if the package is in the ignore packages, just skip
    for ignore_package in doxy_ignore_packages:
        if ignore_package in package:
            skip_package = True

    # If this package is not in the ignore list for doxygen, then just generate its automatic code documentation
    if not skip_package:
        
        # Create the directory to place the doxygen output
        os.makedirs(output_doxygen_directory + '/' + package[1:], exist_ok=True)   
        
        # Run the doxygen output
        result = subprocess.run(
            ['( cat ' + doxyfile_location + ' ; echo "OUTPUT_DIRECTORY=' + output_doxygen_directory + '/' + package[1:] + '" ) | doxygen - '],
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE, 
            text=True,
            cwd=package[1:],
            shell=True)

        print(result.stdout)
        print(result.stderr)

# ----------------------------------------------------------------------
# Incorporate the XML generated by the doxygen in the markdown of mkdocs
# ----------------------------------------------------------------------


# --------------------------------------------------------------------------------------
# Update the main mkdocs.yml file with the list of packages and respective documentation
# --------------------------------------------------------------------------------------

# Create a Yaml parser that preserves the comments and file order
yaml = ruamel.yaml.YAML()

# Open the mkdocs yaml and add the packages that we want our documentation to reference
with open(config_file) as fp:
    data = yaml.load(fp)
    
    # Get the list of items inside the 'nav' yaml tag
    for i, docs in enumerate(data['nav']):

        # Find the 'Packages tag'
        for key, value in docs.items():

            # Add the packages that we want our documentation to reference (according to the ignore_packages file) and the ones discovered by ros
            if key == nav_key:
                docs[key] = packages

# Update the mkdocs yaml file, mainting the comments and order of the yaml data
with open(config_file, "w") as fp:
    yaml.dump(data, fp)