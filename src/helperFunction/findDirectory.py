def find_in_workspaces(search_dirs=None, project=None, path=None, _workspaces=None, considered_paths=None, first_matching_workspace_only=False, first_match_only=False, workspace_to_source_spaces=None, source_path_to_packages=None):
    '''
    Find all paths which match the search criteria.
    All workspaces are searched in order.
    Each workspace, each search_in subfolder, the project name and the path are concatenated to define a candidate path.
    If the candidate path exists it is appended to the result list.
    Note: the search might return multiple paths for 'share' from devel- and source-space.

    :param search_dir: The list of subfolders to search in (default contains all valid values: 'bin', 'etc', 'lib', 'libexec', 'share'), ``list``
    :param project: The project name to search for (optional, not possible with the global search_in folders 'bin' and 'lib'), ``str``
    :param path: The path, ``str``
    :param _workspaces: (optional, used for unit tests), the list of workspaces to use.
    :param considered_paths: If not None, function will append all path that were searched
    :param first_matching_workspace_only: if True returns all results found for first workspace with results
    :param first_match_only: if True returns first path found (supercedes first_matching_workspace_only)
    :param workspace_to_source_spaces: the dictionary is populated with mappings from workspaces to source paths, pass in the same dictionary to avoid repeated reading of the catkin marker file
    :param source_path_to_packages: the dictionary is populated with mappings from source paths to packages, pass in the same dictionary to avoid repeated crawling
    :raises ValueError: if search_dirs contains an invalid folder name
    :returns: List of paths
    '''
    search_dirs = _get_valid_search_dirs(search_dirs, project)
    if 'libexec' in search_dirs:
        search_dirs.insert(search_dirs.index('libexec'), 'lib')
    if _workspaces is None:
        _workspaces = get_workspaces()
    if workspace_to_source_spaces is None:
        workspace_to_source_spaces = {}
    if source_path_to_packages is None:
        source_path_to_packages = {}

    paths = []
    existing_paths = []
    try:
        for workspace in (_workspaces or []):
            for sub in search_dirs:
                # search in workspace
                p = os.path.join(workspace, sub)
                if project:
                    p = os.path.join(p, project)
                if path:
                    p = os.path.join(p, path)
                paths.append(p)
                if os.path.exists(p):
                    existing_paths.append(p)
                    if first_match_only:
                        raise StopIteration

                # for search in share also consider source spaces
                if project is not None and sub == 'share':
                    if workspace not in workspace_to_source_spaces:
                        workspace_to_source_spaces[workspace] = get_source_paths(workspace)
                    for source_path in workspace_to_source_spaces[workspace]:
                        if source_path not in source_path_to_packages:
                            source_path_to_packages[source_path] = find_packages(source_path)
                        matching_packages = [p for p, pkg in source_path_to_packages[source_path].items() if pkg.name == project]
                        if matching_packages:
                            p = source_path
                            if matching_packages[0] != os.curdir:
                                p = os.path.join(p, matching_packages[0])
                            if path is not None:
                                p = os.path.join(p, path)
                            paths.append(p)
                            if os.path.exists(p):
                                existing_paths.append(p)
                                if first_match_only:
                                    raise StopIteration

            if first_matching_workspace_only and existing_paths:
                break

    except StopIteration:
        pass

    if considered_paths is not None:
        considered_paths.extend(paths)

    return existing_paths