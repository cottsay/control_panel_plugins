^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package control_panel_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Use global GLWidget instance
* Require control_panel with global GLWidget functionality
* CPAttitude: Initial plugin
* CPRange: Fix bad label assignment
* CPRange: Plugin configuration fix
* CPImage: Initial plugin
* CPBool: Fix regression in label
* CPRange: Initial plugin
* CPEmptySrv: Joy binding fixes
* CPEmptySrv: Add joy bindings
* Use signals to update GUI elements
* Fix context menu on disabled button widget
  QWidgets ignore all events while disabled.
* Handle validity of service client differently
  For a non-persistent service client, isValid is always true so we
  can no longer use this to determine if the widget is active. Instead,
  we'll just keep track of the validity separately.
* Initial commit
* Contributors: Scott K Logan
