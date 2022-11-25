/**
Copyright 2022 FRC Team 997

This program is free software: 
you can redistribute it and/or modify it under the terms of the 
GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.
*/
package org.chsrobotics.offseasonRobot2022.aprilTags;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/** Object (temporary) representing a group of tags on a field. */
public class TagLayout {
    private final HashMap<Integer, List<AprilTag>> storedTags = new HashMap<>();

    /**
     * Constructs a TagLayout.
     *
     * @param tags The tags to include as part of the layout.
     */
    public TagLayout(AprilTag... tags) {
        for (AprilTag tag : tags) {
            List<AprilTag> tagsAtID = storedTags.get(tag.getID());

            if (tagsAtID != null) {
                tagsAtID.add(tag);
            } else {
                tagsAtID = List.of(tag);
            }

            storedTags.put(tag.getID(), tagsAtID);
        }
    }

    /**
     * Returns whether an AprilTag exists with a specific ID.
     *
     * @param id The fiducial ID to check for.
     * @return If an AprilTag exists at the ID.
     */
    public boolean tagExistsAtID(int id) {
        return storedTags.containsKey(id);
    }

    /**
     * Returns a list of all AprilTags with a specific fiducial ID.
     *
     * @param id The fiducial ID to query.
     * @return A list of all the AprilTags at that ID. May be empty.
     */
    public List<AprilTag> getTagsAtID(int id) {
        return storedTags.get(id);
    }

    /**
     * Returns a list of all AprilTags.
     *
     * @return A list of all the AprilTags in the layout. May be empty.
     */
    public List<AprilTag> getAllTags() {
        List<AprilTag> retVals = new ArrayList<>();

        for (List<AprilTag> tagsAtID : storedTags.values()) {
            retVals.addAll(tagsAtID);
        }

        return retVals;
    }
}
