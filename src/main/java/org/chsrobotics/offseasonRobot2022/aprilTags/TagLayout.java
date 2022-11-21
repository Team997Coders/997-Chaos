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

public class TagLayout {
    private final HashMap<Integer, List<AprilTag>> storedTags = new HashMap<>();

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
     * @param id
     * @return
     */
    public boolean tagExistsAtID(int id) {
        return storedTags.containsKey(id);
    }

    /**
     * @param id
     * @return
     */
    public List<AprilTag> getTagsAtID(int id) {
        return storedTags.get(id);
    }

    /**
     * @return
     */
    public List<AprilTag> getAllTags() {
        List<AprilTag> retVals = new ArrayList<>();

        for (List<AprilTag> tagsAtID : storedTags.values()) {
            retVals.addAll(tagsAtID);
        }

        return retVals;
    }
}
